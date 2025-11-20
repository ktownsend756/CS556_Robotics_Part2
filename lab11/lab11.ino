//Starter Code

#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4Buttons.h>
using namespace Pololu3piPlus32U4;
#include "particle_filter.h"
#include "odometry.h"
#include "Map.h"
#include "sonar.h"
#include "PIDcontroller.h"


Map worldMap; // one map for the program

ButtonA buttonA;
ButtonB buttonB;
ButtonC buttonC;

#define PI 3.14159

#define lenOfMap 60 //60 cm / 3 = 20 cm per grid unit (matches the map).
#define N_particles 25
#define move_noise   0.01   // σ_d^2
#define rotate_noise 0.05   // σ_θ^2
#define ultra_noise  0.10   // σ_s^2

#define diaL 3.2 //define physical robot parameters
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

Motors motorsP;
Encoders encoders;

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio);
ParticleFilter particle(worldMap, lenOfMap, N_particles, move_noise, rotate_noise, ultra_noise);

uint8_t iter =0;
Sonar sonar = Sonar(4);

//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = 10;
float y = 10;
float theta = 0;
float x_last = 0.0;
float y_last = 0.0;
float theta_last = 0.0;


//PID Movement
//feel free to finetune gains and clampings
#define kp 200
#define kd 5
#define ki 0.5
#define clamp_i 50
#define base_speed 100
#define minOutput - 100
#define maxOutput 100

static inline float wrapPi(float a){ while(a <= -PI) a += 2*PI; while(a > PI) a -= 2*PI; return a; }
static inline float angleErr(float target, float curr){ float a = target - curr; while(a <= -PI) a += 2*PI; while(a > PI) a -= 2*PI; return a; }



//Comment out if using motion primitives
PIDcontroller PIDcontroller(kp, ki, kd, minOutput, maxOutput, clamp_i);
double PID_OUT_ANGLE, PID_OUT_DISTANCE;

void setup() {
    Serial.begin(9600);
    while (!Serial) continue;
    delay(1000);

  //seed RNG for diverse particles.  // seed once
  randomSeed(analogRead(A0));  
 
  // //This is an example of how to estimate the distance to a wall for the given
  // //map, assuming the robot is at (0, 0) and has heading PI
  // (Optional quick test; use the global 'map')
  float origin[2] = {0.5, 0.5};
  float theta = 0;
  float closestDist = worldMap.closest_distance(origin, theta);
  Serial.println(closestDist);
}

void loop() {
  sensing_and_movement();


  //Get odometer readings  
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;   
  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);


  //movement function
  //Propagate particles by using move_particles.
  //Parameters are change from current and past odometer values
  //Use qrapPi function to Wrap dθ when propagating particles
  //TODO: Put code under here 
  float c = cos(theta_last), s = sin(theta_last);
  float dx_body = c*(x-x_last) + s*(y - y_last);
  float dy_body = -s*(x - x_last) + c*(y - y_last);


  particle.move_particles(dx_body, dy_body, wrapPi(theta - theta_last));



  //Measaure, estimation, and resample
  //Calculate particle's posterior probabilities, calculate estimated robot's position, and resample
  //TODO: Put code under here 
  particle.measure();
  particle.estimate_position();


  // Display all particle locations and estimated robot location on screen   
  //TODO: Put code under here 
  particle.print_particles();
  
    
  //save last odometer reading
  //TODO: Fill in "..."
  x_last = x;
  y_last = y;
  theta_last = theta;
    
  iter++;

  if (confidence()){
    motorsP.setSpeeds(0,0);
    Serial.println("Confident");
    while(1);
  }
    
 
}



void sensing_and_movement(){
  
  float dist = sonar.readDist();
  const float wallDist = 10.0;
  double dist_err;

  if(dist < wallDist){
    float goal_theta = wrapPi(theta - PI/2.0);
    PID_OUT_ANGLE = PIDcontroller.update(0, angleErr(goal_theta,theta));
    motorsP.setSpeeds(PID_OUT_ANGLE, -PID_OUT_ANGLE);
    delay(1500);
    motorsP.setSpeeds(0,0);
    delay(500);
    if(sonar.readDist() < wallDist){
      motorsP.setSpeeds(PID_OUT_ANGLE, -PID_OUT_ANGLE);
      delay(3000);
      motorsP.setSpeeds(0,0);
      delay(500);
    }
  }
  else{
    float goal_x = x + 10 * cos(theta);
    float goal_y = y + 10 * sin(theta);

    dist_err = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    PID_OUT_DISTANCE = PIDcontroller.update(0, dist_err);
    motorsP.setSpeeds(-PID_OUT_DISTANCE, -PID_OUT_DISTANCE);
    delay(1500);
    motorsP.setSpeeds(0,0);
  }


}


bool confidence(){

  if(iter < 20){
    return false;
  }

  int grid[3][3] = {0};

  for(int i = 0; i < N_particles; i++){
    int x = constrain(particle._particle_list[i].x, 0, 2);
    int y = constrain(particle._particle_list[i].y, 0, 2);
    grid[x][y]++;
  }

  int max = 0;
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      if(grid[i][j] > max){
        max = grid[i][j];
      }
    }
  }

  if(max >=0.88 * N_particles){
    return true;
  }
  else{
    return false;
  }


}

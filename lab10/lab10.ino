//Starter Code

#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4Buttons.h>
using namespace Pololu3piPlus32U4;
#include "particle_filter.h"
#include "odometry.h"
#include "Map.h"
#include "sonar.h"


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

float goal_theta = PI/2; //Goal theta for movement function

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
  movement();

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
  movement();


  //Measaure, estimation, and resample
  //Calculate particle's posterior probabilities, calculate estimated robot's position, and resample
  //TODO: Put code under here 



  // Display all particle locations and estimated robot location on screen   
  //TODO: Put code under here 

  
    
  //save last odometer reading
  //TODO: Fill in "..."
  x_last = x
  y_last = y
  theta_last = theta
    
  iter++;
    
  delay(1000); //for easier viewing of output
 
}

void movement(){
  // Remote-Controlling 
  int leftSpeed = 0, rightSpeed = 0;
  if(buttonA.isPressed()){ //turn left
    //Note, if you use PIDconrollers,know that PID assumes: output = Kp*(setpoint - measured) + ...
    // Using a relative setpoint makes each button press add/subtract 90°
    //your PID call needs a target angle (setpoint), not just “turn right.”
    //PIDcontroller.update(theta, goal_theta) sets a fixed global setpoint of +π/2 (≈ 90°). That means:
    //First left turn from θ≈0 → good (aims for +90°).
    //Next left turn from θ≈+90° → does nothing, because you’re already at the setpoint (+90°). It won’t add another 90°.
    //So, basically, your left turn target = current heading + 90°

    //movement function here
    //TODO: Put code under here
    PID_OUT_ANGLE = PIDcontroller.update(theta, goal_theta);

    leftSpeed = -PID_OUT_ANGLE;
    rightSpeed = PID_OUT_ANGLE;
    motorsP.setSpeeds(leftSpeed, rightSpeed);

    Serial.print("Left pressed!\n");
  } else if(buttonB.isPressed()){ // drive forward
    //movement function here
    //TODO: Put code under here
    


    Serial.print("Forward pressed!\n");
  } else if(buttonC.isPressed()){ // turn right
    //movement function here
    //TODO: Put code under here
    PID_OUT_ANGLE = PIDcontroller.update(theta_last, -goal_theta);

    leftSpeed = PID_OUT_ANGLE;
    rightSpeed = -PID_OUT_ANGLE;
    motorsP.setSpeeds(leftSpeed, rightSpeed);


    Serial.print("Right pressed!\n");
  }
 
}

#include <Pololu3piPlus32U4.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;
#include "odometry.h"
#include "sonar.h"
#include "PIDcontroller.h"


LineSensors lineSensors;
Motors motors;
Servo servo;

Sonar sonar = sonar(4);

#define PI 3.14159

//Physical Robot Parameters 
#define diaL 3.2 
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio);
Encoders encoders;

//Odometry Variables
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = 10;
float y = 10;
float theta = 0;
float x_last = 0.0;
float y_last = 0.0;
float theta_last = 0.0;

//PID Controller Variables
#define kp 200
#define kd 5
#define ki 0.5
#define clamp_i 50
#define base_speed 100
#define minOutput -100
#define maxOutput 100

PIDcontroller PIDcontroller(kp, ki, kd, minOutput, maxOutput, clamp_i);
double PID_OUT_ANGLE, PID_OUT_DISTANCE;

const float wallDist = 10.0;
float frontDist;
float leftDist;

static inline float wrapPi(float a){ while(a <= -PI) a += 2*PI; while(a > PI) a -= 2*PI; return a; }
static inline float angleErr(float target, float curr){ float a = target - curr; while(a <= -PI) a += 2*PI; while(a > PI) a -= 2*PI; return a; }

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(90);
  delay(1000);

}

void loop() {

  sensing_and_movement();

  //Get odometer readings  
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;   
  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);


}


void sensing_and_movement(){
  frontDist = sonar.readDist();
  delay(500);
  servo.write(0);
  delay(500);
  leftDist = sonar.readDist();
  servo.write(90);
  delay(500);

  // case 1: Left wall detected, front wall not detected
  if(leftDist < wallDist && frontDist > wallDist){
    float goal_x = x + 10 * cos(theta);
    float goal_y = y + 10 * sin(theta);

    dist_err = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    PID_OUT_DISTANCE = PIDcontroller.update(0, dist_err);
    motors.setSpeeds(-PID_OUT_DISTANCE, -PID_OUT_DISTANCE);
    delay(1500);
    motors.setSpeeds(0,0);
  }

  // case 2: Left wall detected, front wall detected
  else if(leftDist < wallDist && frontDist < wallDist){
    float goal_theta = wrapPi(theta - PI/2.0);
    PID_OUT_ANGLE = PIDcontroller.update(0, angleErr(goal_theta,theta));
    motors.setSpeeds(PID_OUT_ANGLE, -PID_OUT_ANGLE);
    delay(1500);
    motors.setSpeeds(0,0);
    frontDist = sonar.readDist();
    if(frontDist < wallDist){
      motors.setSpeeds(PID_OUT_ANGLE, -PID_OUT_ANGLE);
      delay(1500);
      motors.setSpeeds(0,0);
      delay(500);
    }
  }

  // case 3: Left wall not detected, front wall detected OR no wall detected
  else{
    float goal_theta = wrapPi(theta - PI/2.0);
    PID_OUT_ANGLE = PIDcontroller.update(0, angleErr(goal_theta,theta));
    motors.setSpeeds(-PID_OUT_ANGLE, PID_OUT_ANGLE);
    delay(1500);
    motors.setSpeeds(0,0);
    
    float goal_x = x + 10 * cos(theta);
    float goal_y = y + 10 * sin(theta);

    dist_err = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    PID_OUT_DISTANCE = PIDcontroller.update(0, dist_err);
    motors.setSpeeds(-PID_OUT_DISTANCE, -PID_OUT_DISTANCE);
    delay(1500);
    motors.setSpeeds(0,0);
  }

  
}


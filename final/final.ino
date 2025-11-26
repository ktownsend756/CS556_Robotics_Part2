/*
* File: final.ino
* Team: 3
* Robot: 28
* Description: Connects various components of the robot such as sonar, servo, and PIDcontrollers
* so the robot can navigate through the maze switching task as needed depending on it's current position
* in the environment
*/

#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4Buzzer.h>
#include <Servo.h>
using namespace Pololu3piPlus32U4;
#include "odometry.h"
#include "sonar.h"
#include "PIDcontroller.h"


//Initialize Robot Components
LineSensors lineSensors;
Motors motors;
Servo servo;
Buzzer buzzer;
Sonar sonar(4);

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
float dist_err;

const float wallDist = 15.0;
float frontDist;
float leftDist;

//Keep angle values between -PI and PI [-PI, PI]
static inline float wrapPi(float a){ while(a <= -PI) a += 2*PI; while(a > PI) a -= 2*PI; return a; }

//Finds the shortest rotation between two angles
static inline float angleErr(float target, float curr){ float a = target - curr; while(a <= -PI) a += 2*PI; while(a > PI) a -= 2*PI; return a; }

//Map layout
int direction = 0; // 0 = down, 1 = left, 2 = up, 3 = right
char grid[4][9] = {
  {'N','N','N','N','N','N','N','N','N'},
  {'N','N','N','N','N','N','N','N','N'},
  {'N','N','N','N','N','N','N','N','N'},
  {'N','N','N','N','N','N','N','N','N'}
  };
int row = 0;
int col = 0;
int cells = 36;

//Movement log array 
char movements[100];
int movement_counter = 0;

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(90);
  delay(10000);

  grid[0][0] = 'V';
}

void loop() {
  
  sensing_and_movement(); 

  //Get odometer readings  
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;   
  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);

  mark_visited();
  cells--;

  //Once every cell has been visited, robot should return to the charging dock
  if(cells <= 0){
    backToDock(movements);
    motors.setSpeeds(0, 0); 
    delay(5000);
  }

}

//Robot uses sensors to read its surrondings then makes the proper movement based on its position in the environment
void sensing_and_movement(){
  frontDist = sonar.readDist();
  delay(500);
  servo.write(180);
  delay(500);
  leftDist = sonar.readDist();
  servo.write(90);
  delay(500);

  // case 1: Left wall detected, front wall not detected
  // move forward
  if(leftDist < wallDist && frontDist > wallDist){
    float goal_x = x + 10 * cos(theta);
    float goal_y = y + 10 * sin(theta);

    dist_err = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    PID_OUT_DISTANCE = PIDcontroller.update(0, dist_err) * 2.5;
    motors.setSpeeds(-PID_OUT_DISTANCE, -PID_OUT_DISTANCE);
    delay(1500);
    motors.setSpeeds(0,0);
    movements[movement_counter] = 'F'; //Log movement
    movement_counter++;
  }

  // case 2: Left wall detected, front wall detected
  // rotate right
  else if(leftDist < wallDist && frontDist < wallDist){
    float goal_theta = wrapPi(theta - PI/2.0);
    PID_OUT_ANGLE = PIDcontroller.update(0, angleErr(goal_theta,theta));
    motors.setSpeeds(PID_OUT_ANGLE, -PID_OUT_ANGLE);
    delay(1500);
    motors.setSpeeds(0,0);
    movements[movement_counter] = 'R'; //Log movement
    movement_counter++;
    frontDist = sonar.readDist();
    direction = (direction + 1) % 4;
    if(frontDist < wallDist){
      motors.setSpeeds(PID_OUT_ANGLE, -PID_OUT_ANGLE);
      delay(1500);
      motors.setSpeeds(0,0);
      movements[movement_counter] = 'R'; //Log movement
      movement_counter++;
      delay(500);
      direction = (direction + 1) % 4;
    }
  }

  // case 3: Left wall not detected, front wall detected OR no wall detected
  // rotate left & move forward
  else{
    float goal_theta = wrapPi(theta - PI/2.0);
    PID_OUT_ANGLE = PIDcontroller.update(0, angleErr(goal_theta,theta));
    motors.setSpeeds(-PID_OUT_ANGLE, PID_OUT_ANGLE);
    delay(1500);
    motors.setSpeeds(0,0);
    movements[movement_counter] = 'L'; //Log movement
    movement_counter++;
    
    float goal_x = x + 10 * cos(theta);
    float goal_y = y + 10 * sin(theta);

    dist_err = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    PID_OUT_DISTANCE = PIDcontroller.update(0, dist_err) * 2.5;
    motors.setSpeeds(-PID_OUT_DISTANCE, -PID_OUT_DISTANCE);
    delay(1500);
    motors.setSpeeds(0,0);
    movements[movement_counter] = 'F'; //Log movement
    movement_counter++;

    direction = (direction + 3) % 4;
  }
  
}

//Marks unvisited cells in the map as visited once the robot reaches them
void mark_visited(){
  // facing down
  if(direction == 0) grid[++row][col] = 'V';
  // facing left
  if(direction == 1) grid[row][--col] = 'V';
  // facing up
  if(direction == 2) grid[--row][col] = 'V';
  // facing right
  if(direction == 3) grid[row][++col] = 'V';

  
  // debugging purposes
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 9; c++) {
        Serial.print(grid[r][c]);
        Serial.print(' ');
    }
    Serial.println();
  }
}

//Robot uses the movement log to retrace its steps back to the charging dock
void backToDock(char _movements[]){
  //Initially turn Robot around 180 degrees to begin backtrack
  float goal_theta = wrapPi(theta - PI);
  PID_OUT_ANGLE = PIDcontroller.update(0, angleErr(goal_theta,theta));
  motors.setSpeeds(PID_OUT_ANGLE, -PID_OUT_ANGLE);
  delay(1500);
  motors.setSpeeds(0,0);
  positionUpdate();

  //Traverse through the movement log and make the proper movement
  for(int i = movement_counter; i >= 0; i--){
    char movement = _movements[i];
    

    if(movement == 'F'){ //Move Forward
      float goal_x = x + 10 * cos(theta);
      float goal_y = y + 10 * sin(theta);
      dist_err = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
      PID_OUT_DISTANCE = PIDcontroller.update(0, dist_err);
      motors.setSpeeds(-PID_OUT_DISTANCE, -PID_OUT_DISTANCE);
      delay(1500);
      motors.setSpeeds(0,0);
      positionUpdate();
    }
    else if(movement == 'L'){ // Opposite -> Rotate Right
      float goal_theta = wrapPi(theta - PI/2.0);
      PID_OUT_ANGLE = PIDcontroller.update(0, angleErr(goal_theta,theta));
      motors.setSpeeds(PID_OUT_ANGLE, -PID_OUT_ANGLE);
      delay(1500);
      motors.setSpeeds(0,0);
      positionUpdate();
    }
    else if(movement == 'R'){ // Opposite -> Rotate Left
      float goal_theta = wrapPi(theta + PI/2.0);
      PID_OUT_ANGLE = PIDcontroller.update(0, angleErr(goal_theta,theta));
      motors.setSpeeds(-PID_OUT_ANGLE, PID_OUT_ANGLE);
      delay(1500);
      motors.setSpeeds(0,0);
      positionUpdate();
    }

  }

  buzzer.playFrequency(1000, 200, 10); //Completion beep

}

//Uses encoder counts and odometry to update the robot's position
void positionUpdate(){
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;   
  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);
}

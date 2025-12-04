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
#include "PDcontroller.h"

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
float x = 20;
float y = 20;
float theta = 0;
float x_last = 0.0;
float y_last = 0.0;
float theta_last = 0.0;

//Robot's Rotation Functionality
//PID Controller Variables (Rotation Controller)
#define kp 200
#define kd 5
#define ki 0.5
#define clamp_i 50
#define minOutput -100
#define maxOutput 100

PIDcontroller PIDcontroller(kp, ki, kd, minOutput, maxOutput, clamp_i);
double PID_OUT_ANGLE, PID_OUT_DISTANCE;
float dist_err;

//Robot's Wall Following Functionality
//PD Controller Variables (Wall Following)
#define minOutput -150     // Minimum correction value
#define maxOutput 150      // Maximum correction value
#define kp_wall 7        // Proportional gain, may need to change
#define kd_wall 3         // Derivative gain, may need to change

#define baseSpeed 200

PDcontroller pd_wall(kp_wall, kd_wall, minOutput, maxOutput);
double PD_OUT_WALL;

const float wallDist = 15.0;
float frontDist;
float leftDist;
float distfromWall;
const float goalDist = 10.0;

//Robot's Line Detection Functionality
//PD Controller Variables (Line Following)
#define minOutput -100      // Minimum correction value
#define maxOutput 100       // Maximum correction value
#define kp_line 0.25        // Proportional gain
#define kd_line 1           // Derivative gain

PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);


int calibrationSpeed = 60; // Speed during calibration rotation
int lineCenter = 2000; // Target position (center of 0-4000 range)

//Line Thresholds
// BLUE line detection range (calibrated values typically 200-500)
const uint16_t BLUE_MIN_CAL = 200;
const uint16_t BLUE_MAX_CAL = 500;
// BLACK square threshold (calibrated values typically > 800)
const uint16_t BLACK_THRESHOLD = 800;

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

//Collected Bins Tracker
bool binsCollected[3] = {false, false, false};

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(90);
  delay(5000);
  //calibrateSensors(); 
  grid[0][0] = 'V';
}

void loop() {
  sensing_and_movement_v2(); 

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
  

  //Declare array for lineSensor values
  uint16_t lineSensorValues[5];
  //Read sensor values
  lineSensors.readCalibrated(*lineSensorValues);
  //Get center sensor's value
  uint16_t centerSensor = lineSensorValues[2];
  //Print output for debugging
  Serial.print("Center Sensor Value: ");
  Serial.println(centerSensor);

  //Check for Black Square (Bin)
  if(centerSensor >= BLACK_THRESHOLD){
    if(!binsCollected[0]){ //Checks if first bin has been collected 
      motors.setSpeeds(-200, 200);
      delay(2500);
      motors.setSpeeds(0, 0);
      binsCollected[0] = true;
      delay(500); //For debugging
    }
    else if(!binsCollected[1]){ //Checks if second bin has been collected
      motors.setSpeeds(-200, 200);
      delay(2500);
      motors.setSpeeds(0, 0);
      binsCollected[1] = true;
      delay(500); //For debugging
    }
    else if(!binsCollected[2]){ //Checks if third bin has been collected
      motors.setSpeeds(-200, 200);
      delay(2500);
      motors.setSpeeds(0, 0);
      binsCollected[2] = true;
      delay(500); //For debugging
    }    
  }
  //Check for Blue Line (Safety Zone -> Max Speed)
  else if(centerSensor >= BLUE_MIN_CAL && centerSensor <= BLUE_MAX_CAL){
    lineFollowing();
      
  }
  

}



void sensing_and_movement_v2(){
  senseWalls();

  // case 1: Left wall detected, front wall not detected
  // move forward
  if(leftDist < wallDist && frontDist > wallDist){
    if(frontDist < 35.0){
      float goal_x = x + 20 * cos(theta);
      float goal_y = y + 20 * sin(theta);

      dist_err = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
      PID_OUT_DISTANCE = PIDcontroller.update(0, dist_err);
      motors.setSpeeds(-PID_OUT_DISTANCE, -PID_OUT_DISTANCE);
      delay(1500);
      motors.setSpeeds(0,0);
      movements[movement_counter] = 'F'; //Log movement
      movement_counter++;
      direction = (direction + 3) % 4;
    } else{
      unsigned long start = millis();

      while (millis() - start < 1300){

        PD_OUT_WALL = pd_wall.update(leftDist, goalDist);
        int leftSpeed = baseSpeed + PD_OUT_WALL;
        int rightSpeed = baseSpeed - PD_OUT_WALL;
        motors.setSpeeds(leftSpeed, rightSpeed); //Robot starts following the wall adjusting it's distance to stay center
        movements[movement_counter] = 'F'; //Log movement
        movement_counter++;
      }
    }
  }
  // case 2: Left wall detected, front wall detected
  // rotate right
  else if(leftDist < wallDist && frontDist < wallDist){
    servo.write(90);
    delay(500);
    float goal_theta = wrapPi(theta - PI/2.0);
    PID_OUT_ANGLE = PIDcontroller.update(0, angleErr(goal_theta,theta));
    motors.setSpeeds(PID_OUT_ANGLE, -PID_OUT_ANGLE);
    delay(1500);
    motors.setSpeeds(0,0); //Initial 90 degree right turn completed
    movements[movement_counter] = 'R'; //Log movement
    movement_counter++;
    frontDist = sonar.readDist();
    direction = (direction + 1) % 4;
    if(frontDist < wallDist){ //Check if there is another wall in front
      motors.setSpeeds(PID_OUT_ANGLE, -PID_OUT_ANGLE);
      delay(1500);
      motors.setSpeeds(0,0); //Do a second 90 degree right turn
      movements[movement_counter] = 'R'; //Log movement
      movement_counter++;
      delay(500);
      direction = (direction + 1) % 4;
    } 
  }
  // case 3: Left wall not detected, front wall detected OR no wall detected
  // rotate left & move forward
  else{
    Serial.println("start");

    servo.write(90);
    delay(500);
    float goal_theta = wrapPi(theta - PI/2.0);
    PID_OUT_ANGLE = PIDcontroller.update(0, angleErr(goal_theta,theta));
    motors.setSpeeds(-PID_OUT_ANGLE, PID_OUT_ANGLE);
    delay(1500);
    motors.setSpeeds(0,0);
    movements[movement_counter] = 'L'; //Log movement
    movement_counter++;
    
    float goal_x = x + 20 * cos(theta);
    float goal_y = y + 20 * sin(theta);

    dist_err = sqrt(pow(goal_x - x, 2) + pow(goal_y - y, 2));
    PID_OUT_DISTANCE = PIDcontroller.update(0, dist_err);
    motors.setSpeeds(-PID_OUT_DISTANCE, -PID_OUT_DISTANCE);
    delay(1500);
    motors.setSpeeds(0,0);
    movements[movement_counter] = 'F'; //Log movement
    movement_counter++;
    direction = (direction + 3) % 4;
  }
}

void senseWalls(){
  Serial.print("Sensing Walls Function Test");
  servo.write(90);
  delay(300);
  frontDist = sonar.readDist();
  servo.write(180);
  delay(300);
  leftDist = sonar.readDist();
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
      unsigned long start = millis();

      while (millis() - start < 1300){

        PD_OUT_WALL = pd_wall.update(leftDist, goalDist);
        int leftSpeed = baseSpeed + PD_OUT_WALL;
        int rightSpeed = baseSpeed - PD_OUT_WALL;
        motors.setSpeeds(leftSpeed, rightSpeed); //Robot starts following the wall adjusting it's distance to stay center
        positionUpdate();
      }
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

//Phase 3 & 4 (IR Sensors/Line Following)
void calibrateSensors()
{
  delay(1000);
  Serial.println("Calibrating sensors...");   
  
  for(int i = 0; i < 120; i++){
    if (i > 30 && i <= 90){
      // Rotate left
      motors.setSpeeds(int(-1 * calibrationSpeed), calibrationSpeed);
    }
    else{
      // Rotate right
      motors.setSpeeds(calibrationSpeed, int(-1 * calibrationSpeed));
    }
    lineSensors.calibrate();
  } 
  
  motors.setSpeeds(0, 0);
  Serial.println("Calibration complete!");
}

bool computeBlueLinePosition(uint16_t cal[5], uint16_t &position){
  // Accumulators for weighted-average calculation
  uint32_t weightedSum = 0;   // Sum of (activation * sensor_position)
  uint32_t total = 0;         // Sum of activations

  // Check each of the 5 sensors
  for(int i = 0; i < 5; i++){
    uint16_t val = cal[i];    // Calibrated value for sensor i 

    // Only consider sensors reading in the "blue" range
    if(val >= BLUE_MIN_CAL && val <= BLUE_MAX_CAL){
      // Calculate activation strength (how "blue" the reading is)
      // Higher values within the blue range = stronger activation
      uint16_t v = val - BLUE_MIN_CAL;
      
      // Calculate this sensor's position on 0-4000 scale
      // Sensor 0 = position 0, Sensor 1 = 1000, etc.
      uint16_t pos = i * 1000;

      // Add this sensor's contribution to weighted sum
      weightedSum += (uint32_t)v * pos;
      total += v;
    }
  }

  // If no sensor detected blue, return false
  if(total == 0){
    return false;
  }

  // Calculate weighted average position
  position = weightedSum / total;
  return true;
}

void lineFollowing(){
  // Read current calibrated sensor values
  uint16_t cal[5];
  lineSensors.readCalibrated(cal);
  
  // Try to find the blue line position
  uint16_t position;
  bool found = computeBlueLinePosition(cal, position);

  // If blue line not detected, stop and return
  if(!found){
    Serial.println("BLUE LINE LOST - Stopping");
    motors.setSpeeds(0, 0);
    return;
  }

  // Calculate PD correction based on position error
  // If position < 2000: line is left of center, need to turn left
  // If position > 2000: line is right of center, need to turn right
  int PDout = pd_line.update(position, lineCenter);

  // Apply correction to motor speeds
  // PDout > 0 means line is right, so speed up left motor
  // PDout < 0 means line is left, so speed up right motor
  int leftSpeed = 300 + PDout;
  int rightSpeed = 300 - PDout;
  
  motors.setSpeeds(leftSpeed, rightSpeed);

  // Debug output
  Serial.print("FOLLOWING - Pos: ");
  Serial.print(position);
  Serial.print(" PD: ");
  Serial.print(PDout);
  Serial.print(" L: ");
  Serial.print(leftSpeed);
  Serial.print(" R: ");
  Serial.println(rightSpeed);
}

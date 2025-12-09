/*
* File: final_KT.ino
* Team: 3
* Robot: 6
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
float x = 10;
float y = 10;
float theta = 0;
float x_last = 0.0;
float y_last = 0.0;
float theta_last = 0.0;

//Robot's Wall Following Functionality
//PD Controller Variables (Wall Following)
#define minOutput -100    // Minimum correction value
#define maxOutput 100     // Maximum correction value
#define kp_wall 7        // Proportional gain, may need to change
#define kd_wall 3        // Derivative gain, may need to change

#define baseSpeed 200

PDcontroller pd_wall(kp_wall, kd_wall, minOutput, maxOutput);
double PD_OUT_WALL;

const float wallDist = 15.0; //Threshold to detect if there is a wall.
float frontDist; //Distance away from front wall
float leftDist; //Distance away from left wall
float distFromWall; 
const float goalDist = 7.0; //Goal distance for wall following

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
//Center sensor calibrated value
uint16_t centerSensor;
//Line Color checks
bool isBlack, isBlue;

//Collected Bins Tracker
bool binsCollected[3] = {false, false, false};


void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(90);
  delay(2500);
  calibrateSensors(); 
}

void loop() {
  //update Odometry
  positionUpdate();

  //Declare array for lineSensor values
  uint16_t lineSensorValues[5];
  //Read sensor values
  lineSensors.readCalibrated(lineSensorValues);
  //Get center sensor's value
  centerSensor = lineSensorValues[2];
  Serial.print("Center Sensor Value: ");
  Serial.println(centerSensor);

  //Reset color flags
  isBlack = false;
  isBlue = false;

  //Check IR sensor
  if(centerSensor >= BLACK_THRESHOLD){
    isBlack = true;
  }
  else if(centerSensor >= BLUE_MIN_CAL && centerSensor <= BLUE_MAX_CAL){
    isBlue = true;
  }

  //Perform corresponding task from sensor read
  if(isBlack){
    binCollection();
  }
  else if(isBlue){
    lineFollowing();
  }
  else{
    wallFollowing();
  }
  
}

//Uses encoder counts and odometry to update the robot's position
void positionUpdate(){
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;   
  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta);
}

void wallFollowing(){
    Serial.println("Following Wall");

    servo.write(180); //Turn servo left
    delay(300); //allow robot to adjust
    //Follow Left Wall
    unsigned long start = millis();
    while (millis() - start < 1100){ //Follow Wall for 20cm
      distFromWall = sonar.readDist(); //Get distance from the wall
      Serial.print("distFromWall: ");
      Serial.println(distFromWall);

      PD_OUT_WALL = pd_wall.update(distFromWall, goalDist);
      int leftSpeed = baseSpeed - PD_OUT_WALL;
      Serial.print("leftSpeed: ");
      Serial.println(leftSpeed);

      int rightSpeed = baseSpeed + PD_OUT_WALL;
      Serial.print("rightSpeed: ");
      Serial.println(rightSpeed);

      motors.setSpeeds(leftSpeed, rightSpeed);
    }

    //Check Front Wall
    servo.write(90); //Turn servo forward
    delay(300);
    frontDist = sonar.readDist();
    if(frontDist <= 10){ //Check if wall in front is less than 10cm away
      motors.setSpeeds(0, 0); //halt robot
      delay(150);
      while(frontDist <= 10){
        motors.setSpeeds(150, -150); //Rotate Right until front wall not detected
        delay(60); //buffer for sonar reads
        frontDist = sonar.readDist();
        
      }
      motors.setSpeeds(0, 0); //Prepare to transition back to wall following
      delay(150);
    }   
}

void binCollection(){
  if(!binsCollected[0]){ //Checks if first bin has been collected 
      motors.setSpeeds(-200, 200);
      delay(1500);
      motors.setSpeeds(0, 0);
      binsCollected[0] = true;
      delay(500); //For debugging
    }
    else if(!binsCollected[1]){ //Checks if second bin has been collected
      motors.setSpeeds(-200, 200);
      delay(1500);
      motors.setSpeeds(0, 0);
      binsCollected[1] = true;
      delay(500); //For debugging
    }
    else if(!binsCollected[2]){ //Checks if third bin has been collected
      motors.setSpeeds(-200, 200);
      delay(1500);
      motors.setSpeeds(0, 0);
      binsCollected[2] = true;
      delay(500); //For debugging
    }    
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
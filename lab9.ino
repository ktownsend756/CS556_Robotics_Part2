#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h" //Uncomment after importing your PDcontroller files

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;
Servo servo;

Sonar sonar(4);

#define minOutput -100
#define maxOutput 100
#define baseSpeed 100
#define kp_line 0
#define kd_line 0
#define kp_obs 0
#define kd_obs 0

PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);
PDcontroller pd_obs(kp_obs, kd_obs, minOutput, maxOutput);

//Recommended Variables

//Calibration
int calibrationSpeed;
unsigned int lineSensorValues[5];
unsigned int lineDetectionValues[5];

//Line Following
int lineCenter = 2000;
int16_t robotPosition;
bool isOnBlack;

//Wall Following
int PDout;
float wallDist;
int distFromWall = 10;

int task = 0; // Line Following = 0 / Wall Following = 1

void calibrateSensors()
{
  //Implement calibration for IR Sensors
  
  for(int i = 0; i<140; i++){
    if(i > 25 && i < 95){
      motors.setSpeeds(-100, 100); // Pivot counter-clockwise
    }
    else{
      motors.setSpeeds(100, -100); // Pivot clockwise
    }
    lineSensors.calibrate();
  }
  motors.setSpeeds(0, 0); //Halts robot after calibration  
}

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(90); // turn servo forward
  delay(2000);

  calibrateSensors();
  
}

void loop(){

 /*
  Hint:
  If there is an object in front of you, switch to wall following
  If there is no object, do line following
  Use if statement such as if(task == 0), do this... if(task == ...), do this...
  For each task, print what task is being done for debugging purposes.
 */

  // line following
  if(task == 0){
    Serial.println("Task: Line Following");
    lineFollowing();

  }
  // wall following
  else if(task == 1){
    Serial.println("Task: Wall Following");
    wallFollowing();
  }

}

//Recommended helper functions
//Uncomment if you choose to implement these functions, but also
//feel free to create your own solutions!


void lineFollowing()
{
  //Get robot's actual position
  robotPosition = lineSensors.readLineBlack(lineSensorValues);
  Serial.print("Robot's position: ");
  Serial.println(robotPosition);

  //Calculate error and adjustment needed
  calibrationSpeed = pd_line.update(robotPosition, lineCenter);

  //Add adjustment to base speed
  int leftSpeed = baseSpeed - calibrationSpeed;
  int rightSpeed = baseSpeed + calibrationSpeed;

  motors.setSpeeds(leftSpeed, rightSpeed);

  detectObject();
}

void detectObject(){
  // sonar reads distance of object
  float dist = sonar.readDist();
    if(dist > 0 && dist < 20){ // if object is between 0-20 it stops and claims it detected an object
      motors.setSpeeds(0,0);
      Serial.println("Obstacle Detected");
      delay(1000);
      task = 1; // switch to wall following
    }
}

void wallFollowing()
{
  //Hint: Your robot shouldn't only be following the wall. It should also be looking
  //      for something else while following the wall.
  servo.write(45); // turn servo to read wall distance
  wallDist = sonar.readDist();
  servo.write(90);

  if (wallDist <=0) return; // error check

  float error = distFromWall - wallDist; // calculate error from real and goal
  PDout = pd_obs.update(error,0); // pdcontroller with error and target

  // set wheel speeds
  int left = baseSpeed + PDout;
  int right = baseSpeed - PDout;
  motors.setSpeeds(left,right);

  detectBlackLine();

}

void detectBlackLine()
{
  lineSensors.read(lineDetectionValues);

    // Threshold value to detect black (adjust based on calibration)
    const int blackThreshold = 1500; 

    // Check if the robot is on a black square
    for (int i = 0; i < 5; i++) {
        Serial.println(lineDetectionValues[i]);
        if (lineDetectionValues[i] > blackThreshold) {
            //#TODO If using this function, decide what to do 
            //      if black line is detected again
            // states line is detected, stops, and switches task to line follow
            motors.setSpeeds(0,0);
            Serial.println("Line Detected");
            task = 0;
            break;
        }
    }
}


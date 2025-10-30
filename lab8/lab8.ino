#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h" //Uncomment after you import your PDcontroller files

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;

#define minOutput -100
#define maxOutput 100
#define baseSpeed 100
#define kp_line 1
#define kd_line .5

PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);

//Recommended Variables

//Calibration
int calibrationSpeed;
unsigned int lineSensorValues[5];

//Line Following
int lineCenter = 2000;
int16_t robotPosition;


void calibrateSensors()
{
  //TASK 2.1a (DONE)
  //Implement calibration for IR Sensors
  //Hint: Have your robot turn to the left and right to calibrate sensors.
  
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
  delay(2000);

  calibrateSensors();
  
}

void loop(){
  //Task 2.1b (DONE)
  //Implement Controller and logic for line following
  //Hint: The actual structure should be similar to you wall following code, 
  //      but instead of sonar you are using the Line Sensors class.
  //Consider making this its own function for easier use in the next lab
  
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

}

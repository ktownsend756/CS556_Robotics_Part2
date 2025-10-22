#include <Pololu3piPlus32U4.h>
//#include "PDcontroller.h" //Uncomment after you import your PDcontroller files

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;

#define minOutput -100
#define maxOutput 100
#define baseSpeed 100
#define kp_line ...
#define kd_line ...

PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);

//Recommended Variables

//Calibration
int calibrationSpeed;
unsigned int lineSensorValues[5];

//Line Following
int lineCenter;
int16_t = robotPosition;


void calibrateSensors()
{
  //TASK 2.1a
  //Implement calibration for IR Sensors
  //Hint: Have your robot turn to the left and right to calibrate sensors.
}

void setup() {
  Serial.begin(9600);
  delay(2000);

  calibrateSensors();
  
}

void loop(){

  //Task 2.1b
  //Implement Controller and logic for line following
  //Hint: The actual structure should be similar to you wall following code, 
  //      but instead of sonar you are using the Line Sensors class.
  //Consider making this its own function for easier use in the next lab
    
}

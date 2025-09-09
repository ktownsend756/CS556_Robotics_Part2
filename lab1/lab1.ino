#include <Pololu3piPlus32U4.h>
#include "my_robot.h"
using namespace Pololu3piPlus32U4;

void setup() {
  Serial.begin(9600);
  delay(15);
}

void loop(){
    /*
    //Code for Task 2: Implement Motion Primitives (DONE)
    //Sets the speeds for motors 
    //The value of the input is given in mm/s with a maximum value of 400
    
    //move forward
    moveForward(.5, .2);
    delay(3000);

    //move backward
    moveBackward(.5, .2);
    delay(3000);

    //turn right(in place)
    turnRight(2, .2);
    delay(3000);

    //turn left (in place)
    turnLeft(2, .2);
    delay(3000);

    //move forward while turning right
    moveForwardTurningRight(.5, .2);
    delay(3000);

    //move forward while turning left
    moveForwardTurningLeft(.5, .2);
    delay(3000);

    //move backward while turning right
    moveBackwardTurningRight(.5, .2);
    delay(3000);

    //move backward while turning left
    moveBackwardTurningLeft(.5, .2);
    delay(3000);

    //halt
    halt();
    delay(5000);
    */

    
    /*
    //Code for Task 3: Classes and Modules

    //Initialize Robot object and set base speed
    MyRobot robot(.2);

    //move forward
    robot.forward(1);
    delay(1000);

    //turn right(in place)
    robot.turn_right(2);
    delay(1000);

    //turn left (in place)
    robot.turn_left(2);
    delay(1000);

    //move backwards
    robot.backward(1);
    delay(1000);

    //halt
    robot.halt();
    delay(3000);
    */

    
}

/* Code for Task 2: Implement Motion Primitives (DONE)
// Makes the robot turn in place to the left for a specified duration at a specified speed
void turnLeft(float duration, float speed){
  int speedval = (int)(speed * 1000); //Convert m/s to mm/s
  int time = (int)(duration * 1000); //Convert s to ms

  Motors::setSpeeds(-speedval, speedval); // set speeds to turn left in place
  delay(time); // delays for given duration
  halt();
}

// Makes the robot turn in place to the right for a specified duration at a specified speed
void turnRight(float duration, float speed){
  int speedval = (int)(speed * 1000); //Convert m/s to mm/s
  int time = (int)(duration * 1000); //Convert s to ms

  Motors::setSpeeds(speedval, -speedval); // set speeds to run right in place
  delay(time); // delays for given duration
  halt();
}

// Makes the robot stay still (until another primitive function is called again)
void halt(){
  Motors::setSpeeds(0, 0); // set speeds to halt
}

// Makes the robot move straight forward for a specified distance at a specified speed
void moveForward(float distance, float speed){
  int speedval = (int)(speed * 1000); //Convert m/s to mm/s
  int time = (int)((distance/speed) * 1000); //Convert s to ms

  Motors::setSpeeds(speedval, speedval); // set speeds to move forward
  delay(time); // delays for given time calculated with distance
  halt();
}

// Makes the robot move straight backward for a specified distance at a specified speed
void moveBackward(float distance, float speed){
  int speedval = (int)(speed * 1000); //Convert m/s to mm/s
  int time = (int)((distance/speed) * 1000); //Convert s to ms

  Motors::setSpeeds(-speedval, -speedval); // set speeds to move backward
  delay(time); // delays for given time calculated with distance
  halt();
}

//  Makes the robot move forward while turning left for a specified distance at a specified speed
void moveForwardTurningLeft(float distance, float speed){
  int rightspeedval = (int)(speed * 1000); //Convert m/s to mm/s
  int leftspeedval = (int)(.75 * speed * 1000); //Inside wheel is at 75% speed of outside wheel (smooth turn)
  int time = (int)((distance/speed) * 1000); //Convert s to ms

  Motors::setSpeeds(leftspeedval, rightspeedval); // set speeds to move forward while turning left
  delay(time); // delays for given time calculated with distance
  halt();
}

// Makes the robot move forward while turning right for a specified distance at a specified speed
void moveForwardTurningRight(float distance, float speed){
  int leftspeedval = (int)(speed * 1000); //Convert m/s to mm/s
  int rightspeedval = (int)(.75 * speed * 1000); //Inside wheel is at 75% speed of outside wheel (smooth turn)
  int time = (int)((distance/speed) * 1000); //Convert s to ms

  Motors::setSpeeds(leftspeedval, rightspeedval); // set speeds to move forward while turning right
  delay(time); // delays for given time calculated with distance
  halt();
}

// Makes the robot move backward while turning left for a specified distance at a specified speed
void moveBackwardTurningLeft(float distance, float speed){
  int rightspeedval = (int)(speed * 1000); //Convert m/s to mm/s
  int leftspeedval = (int)(.75 * speed * 1000); //Inside wheel is at 75% speed of outside wheel (smooth turn)
  int time = (int)((distance/speed) * 1000); //Convert s to ms

  Motors::setSpeeds(-leftspeedval, -rightspeedval); // set speeds to move backward while turning left
  delay(time); // delays for given time calculated with distance
  halt();
}

// Makes the robot move backward while turning right for a specified distance at a specified speed
void moveBackwardTurningRight(float distance, float speed){
  int leftspeedval = (int)(speed * 1000); //Convert m/s to mm/s
  int rightspeedval = (int)(.75 * speed * 1000); //Inside wheel is at 75% speed of outside wheel (smooth turn)
  int time = (int)((distance/speed) * 1000); //Convert s to ms

  Motors::setSpeeds(-leftspeedval, -rightspeedval); // set speeds to move backward while turning right
  delay(time); // delays for given time calculated with distance
  halt();
}
*/




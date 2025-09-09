#include <Pololu3piPlus32U4.h>
#include "my_robot.h"
using namespace Pololu3piPlus32U4;

Motors motors;

//Constructor - takes float variable as parameter to set the base speed of the robot
MyRobot::MyRobot(float base_speed) {
    this->base_speed = base_speed;
}

//Robot moves forward a given distance using base speed
void MyRobot::forward(int distance){
    int speedval = (int)(base_speed*1000); //Convert m/s to mm/s
    int time = (int)((distance/base_speed) * 1000); //Convert s to ms

    motors.setSpeeds(speedval, speedval); // set speeds to move forward
    delay(time); // delays time calculated from distance
    halt();
}

//Robot moves backward a given distance using base speed
void MyRobot::backward(int distance){
    int speedval = (int)(base_speed*1000); //Convert m/s to mm/s
    int time = (int)((distance/base_speed) * 1000); //Convert s to ms

    motors.setSpeeds(-speedval, -speedval); // set speeds to move backward
    delay(time); // delays time calculated from distance
    halt();
}

//Robot turns left for a given duration using base speed (in place)
void MyRobot::turn_left(int duration){
    int speedval = (int)(base_speed*1000); //Convert m/s to mm/s
    int time = (int)(duration * 1000); //Convert s to ms

    motors.setSpeeds(-speedval, speedval); // set speeds to turn left in place
    delay(time); // delays given duration
    halt();
}

//Robot turns right for a given duration using base speed (in place)
void MyRobot::turn_right(int duration){
    int speedval = (int)(base_speed*1000); //Convert m/s to mm/s
    int time = (int)(duration * 1000); //Convert s to ms

    motors.setSpeeds(speedval, -speedval); // set speeds to turn right in place
    delay(time); // delays given duration
    halt();
}

//Robot comes to a complete stop
void MyRobot::halt(){
    motors.setSpeeds(0, 0); // set speeds to halt
}

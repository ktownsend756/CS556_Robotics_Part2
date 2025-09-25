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
    //NEW implementation with millis instead of delay
    int speedval = (int)(base_speed*1000); //Convert m/s to mm/s
    int time = (int)((distance/base_speed) * 1000); //Convert s to ms
    motors.setSpeeds(speedval, speedval);

    unsigned long startTime = millis();
    unsigned long elapsedTime = 0;
    unsigned long duration = (unsigned long) time;

    while(elapsedTime < duration){
        elapsedTime = millis() - startTime;
    }
    halt();
}

//Robot moves backward a given distance using base speed
void MyRobot::backward(int distance){
    //NEW implementation with millis instead of delay
    int speedval = (int)(base_speed*1000); //Convert m/s to mm/s
    int time = (int)((distance/base_speed) * 1000); //Convert s to ms
    motors.setSpeeds(-speedval, -speedval);

    unsigned long startTime = millis();
    unsigned long elapsedTime = 0;
    unsigned long duration = (unsigned long) time;

    while(elapsedTime < duration){
        elapsedTime = millis() - startTime;
    }
    halt();
}

//Robot turns left for a given duration using base speed (in place)
void MyRobot::turn_left(float duration){
    //NEW implementation with millis instead of delay
    int speedval = (int)(base_speed*1000); //Convert m/s to mm/s
    int time = (int)(duration * 1000); //Convert s to ms
    motors.setSpeeds(-speedval, speedval);

    unsigned long startTime = millis();
    unsigned long elapsedTime = 0;
    unsigned long interval = (unsigned long) time;

    while(elapsedTime < interval){
        elapsedTime = millis() - startTime;
    }
    halt();
}

//Robot turns right for a given duration using base speed (in place)
void MyRobot::turn_right(float duration){
    //NEW implementation with millis instead of delay
    int speedval = (int)(base_speed*1000); //Convert m/s to mm/s
    int time = (int)(duration * 1000); //Convert s to ms
    motors.setSpeeds(speedval, -speedval);

    unsigned long startTime = millis();
    unsigned long elapsedTime = 0;
    unsigned long interval = (unsigned long) time;

    while(elapsedTime < interval){
        elapsedTime = millis() - startTime;
    }
    halt();  
}

//Robot comes to a complete stop
void MyRobot::halt(){
    motors.setSpeeds(0, 0);
}

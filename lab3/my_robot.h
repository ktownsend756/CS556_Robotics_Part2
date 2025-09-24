#ifndef my_robot_h
#define my_robot_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class MyRobot{
  public:

    //Constructor with parameter for base speed
    MyRobot(float base_speed);

    //Motion Primitive Functions
    //Robot moves forward a given distance using base speed
    void forward(int distance);

    //Robot moves backward a given distance using base speed
    void backward(int distance);

    //Robot turns left (in place) for a given duration using base speed
    void turn_left(float duration);

    //Robot turns right (in place) for a given duration using base speed
    void turn_right(float duration);

    //Robot comes to a complete stop
    void halt();
    
  private:
    float base_speed;
};

#endif


#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

PDcontroller::PDcontroller(float kp, float kd, double minOutput, double maxOutput) {
  // initialize the private varaibles from Pcontroller.h here
  _kp = kp;
  _kd = kd;
  _minOutput = minOutput;
  _maxOutput = maxOutput;
  _error = 0;
  _prevError = 0;
  _derivative = 0;
  _output = 0;
  _clampedOutput = 0;
  _dampedOutput = 0;
  
}

double PDcontroller::update(double value, double target_value, double lastError){
  //Controller math here
  /*Hints: To add damping (derivative), you must have something to
           keep track of time for the rate of change.
           
           Also note that the first time PD controller is ran, we only have
           the P component, so consider using an if-else statement.

           Again, you need to return actuator controller value (_clampOut)
  */
  //Store error by finding difference between goal distance from wall and actual distance
  _error = target_value - value;

  //Multiply the proportional gain and the error to determine how the robot should fix itself
  _output = _kp * _error;

  //Clamp the output so it does not exceed or drop below the min and max values
  _clampedOutput = constrain(_output, _minOutput, _maxOutput);

}

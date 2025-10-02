#include <Pololu3piPlus32U4.h>
#include "Pcontroller.h"
using namespace Pololu3piPlus32U4;

Pcontroller::Pcontroller(float kp, double minOutput, double maxOutput) {
  // initialize the private varaibles from Pcontroller.h here

  _kp = kp; //Constant gain value to determine how strongly the robot corrects itself
  _minOutput = minOutput; //Minimum clamping value (controller can't return a value below this)
  _maxOutput = maxOutput; //Maximum clamping value (controller can't return a value above this)
  _error = 0;
  _output = 0;
  _clampedOutput = 0;

}

double Pcontroller::update(double value, double target_value){
  //Controller math here
  //Hint: Need to return actuator controller value (_clampOut)
  
  //Store error by finding difference between goal distance from wall and actual distance
  _error = target_value - value;

  //Multiply the proportional gain and the error to determine how the robot should fix itself
  _output = _kp * _error;

  //Clamp the output so it does not exceed or drop below the min and max values
  _clampedOutput = constrain(_output, _minOutput, _maxOutput);

  return _clampedOutput;
}

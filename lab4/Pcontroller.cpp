#include <Pololu3piPlus32U4.h>
#include "Pcontroller.h"
using namespace Pololu3piPlus32U4;

Pcontroller::Pcontroller(float kp, double minOutput, double maxOutput) {
  // initialize the private varaibles from Pcontroller.h here
  _kp = kp;
  _minOutput = minOutput;
  _maxOutput = maxOutput;
  _error = 0;
  _output = 0;
  _clampedOutput = 0;

}

double Pcontroller::update(double value, double target_value){
  //Controller math here
  //Hint: Need to return actuator controller value (_clampOut)
  _error = target_value - value;
  _output = _kp * _error;
  _clampedOutput = constrain(_output, _minOutput, _maxOutput);
  return _clampedOutput;

}

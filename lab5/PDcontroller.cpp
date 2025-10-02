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
  _output = 0;
  _clampedOutput = 0;
  _prevError = 0;
  _prevTime = 0;
  _derivative = 0;
}

double PDcontroller::update(double value, double target_value){
  //Controller math here
  /*Hints: To add damping (derivative), you must have something to
           keep track of time for the rate of change.
           
           Also note that the first time PD controller is ran, we only have
           the P component, so consider using an if-else statement.

           Again, you need to return actuator controller value (_clampOut)
  */
  _error = target_value - value;
  long currentTime = millis();
  double time = (currentTime - _prevTime) / 1000;

  if(_prevTime == 0){
    _output = _kp * _error;
  } else{
    _derivative = (_error - _prevError) / time;
    _output = (_kp*_error) + (_kd*_derivative);
  }

  if(_output > _maxOutput) 
    _clampedOutput = _maxOutput;
  else if (_output < _minOutput) 
    _clampedOutput = _minOutput;
  else 
    _clampedOutput = _output;

  _prevError = _error;
  _prevTime = currentTime;
  return _clampedOutput;
}


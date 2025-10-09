#include <Pololu3piPlus32U4.h>
#include "PIDcontroller.h"
using namespace Pololu3piPlus32U4;

PIDcontroller::PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i) {
  /*Initialize values by copying and pasting from PD controller, then declaring for
  the three new variables.*/
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _minOutput = minOutput;
  _maxOutput = maxOutput;
  _clamp_i = clamp_i;
  _error = 0;
  _prevError = 0;
  _prevTime = 0;
  _derivative = 0;
  _accumulatedError = 0;
  _output = 0;
  _clampedOutput = 0;
}

double PIDcontroller::update(double value, double target_value){
  /*Now copy and paste your PD controller. To implement I component,
  keep track of accumulated error, use your accumulated error in the constrain
  function for the integral, multiply ki by your integral, then add your p, d,
  and i components.
  
  Note: Do not just put all of the integral code at the end of PD component. Think
  about step by step how you can integrate these parts into your PDController
  code.*/

  _error = target_value - value; // error difference
  long currentTime = millis();
  double dt = (currentTime - _prevTime) / 1000; // change of time in seconds

  
  if(_prevTime == 0){
    _output = _kp * _error; // no derivative on first update
  } else{
    _accumulatedError += _error * dt;
    _accumulatedError = constrain(_accumulatedError, -_clamp_i, _clamp_i);
    _derivative = (_error - _prevError) / dt; // de/dt
    _output = (_kp*_error) + (_ki * _accumulatedError) + (_kd*_derivative); // add together
  }

  // constrain to clamp output
  _clampedOutput = constrain(_output, _minOutput, _maxOutput);

  // update prev values
  _prevError = _error;
  _prevTime = currentTime;
  return _clampedOutput;
  
}

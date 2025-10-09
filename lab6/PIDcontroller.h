#ifndef PIDcontroller_h
#define PIDcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PIDcontroller{
  public:
    PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i);
    double update(double value, double target_value);
    
  private:
    /*Add variables from your PDController, then add varaibles for ki, your clamp
    for i component, and accumulated error.*/

    float _kp;
    float _kd;
    double _minOutput;
    double _maxOutput;
    double _error;
    double _prevError;
    double _prevTime;
    double _derivative;
    double _output;
    double _clampedOutput;

    // added variables
    float _ki;
    double _clamp_i;
    double _accumulatedError;


};

#endif

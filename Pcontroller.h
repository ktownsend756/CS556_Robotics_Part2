#ifndef Pcontroller_h
#define Pcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class Pcontroller{
  public:
    Pcontroller(float kp, double minOutput, double maxOutput);
    double update(double value, double target_value); 
    
  private:
    //Add private variables here
    //Hint: There should be 6 variables in total
    float _kp;
    double _minOutput;
    double _maxOutput;
    double _error;
    double _output;
    double _clampedOutput;
	
};

#endif

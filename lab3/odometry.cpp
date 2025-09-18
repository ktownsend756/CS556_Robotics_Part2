#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>
#include "odometry.h"
#include "printOLED.h"
using namespace Pololu3piPlus32U4;

#define PI 3.14159

PrintOLED printOLED;

Odometry::Odometry(float diaL, float diaR, float w, int nL, int nR, int gearRatio, bool dead_reckoning){
  _diaL = diaL;
  _diaR = diaR;
  _w = w;
  _nL = nL;
  _nR = nR;
  _gearRatio = gearRatio;
  _deadreckoning = dead_reckoning;

  _x = 0;
  _y = 0;
  _theta = 0;

  _left_encoder_counts_prev = 0;
  _right_encoder_counts_prev = 0;

  if(_deadreckoning){ // if using dead reckoning, initialize and calibrate IMU
    Wire.begin();
    _imu.init();
    _imu.enableDefault();

    //calibrate IMU
    int total = 0;
    for (int i = 0; i < 100; i++)
    {
      _imu.readGyro();
      total += _imu.g.z;
      delay(1);
    }
    _IMUavg_error = total / 100;  
  }
}

// USE ODOMETRY FORMULAS TO CALCULATE ROBOT'S NEW POSITION AND ORIENTATION
void Odometry::update_odom(int left_encoder_counts, int right_encoder_counts, float &x, float &y, float &theta){
	
  // IF USING dead reckoning, GET THE ANGLE _theta FROM IMU
  
  // OTHERWISE, CALCULATE THE ANGLE _theta FROM ENCODERS DATA BASED ON THE FORMULA FROM THE LECTURES
  int NL = _nL * _gearRatio;
  int NR = _nR * _gearRatio;

  int left = left_encoder_counts - _left_encoder_counts_prev;
  int right = right_encoder_counts - _right_encoder_counts_prev;

  float deltaL = (PI * _diaL * left) / NL;
  float deltaR = (PI * _diaR * right) / NR;

  _theta += (deltaR - deltaL) / _w;
  

  // CALCULATE _x BASED ON THE FORMULA FROM THE LECTURES
  float deltaX = ((deltaL + deltaR) / 2) * cos(_theta);
  _x += deltaX;
  // CALCULATE _y BASED ON THE FORMULA FROM THE LECTURES
  float deltaY = ((deltaL + deltaR) / 2) * sin(_theta);
  _y += deltaY;
  
  // CALCULATE CUMULATIVE x, AND CUMULATIVE y. 
  //AKA UPDATE THE VALUE OF &x AND &y (THE PARAMETERS OF THE update_odom FUNCTIONS, WHICH ARE PASSED BY REFERENCE)
  //REMINDER: CUMULATIVE theta IS EQUAL TO _theta.
  x = _x;
  y = _y;
  theta = _theta;

  // PRINT THE x, y, theta VALUES ON OLED
  printOLED.print_odom(x, y, theta);
  // PRINT THE x, y, theta VALUES ON SERIAL MONITOR
      // Odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta); // x, y, theta call by reference
  Serial.print(" x:  "); 
  Serial.println(x); 
  Serial.print(" y:  ");
  Serial.println(y);
  Serial.print(" theta:  ");
  Serial.println(theta);

  // Save the current encoder values as the "previous" values, so you can use it in the next iteration
  _left_encoder_counts_prev = left_encoder_counts;
  _right_encoder_counts_prev = right_encoder_counts;

}

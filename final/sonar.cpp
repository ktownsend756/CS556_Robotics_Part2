/*
* File: sonar.cpp
* Team: 3
* Robot: 28
* Description: Uses the sonar sensor on the robot to emit and detect soundwaves and read the distance
* between the robot and near by walls/obstacles
*/

#include <Pololu3piPlus32U4.h>
#include "sonar.h"
using namespace Pololu3piPlus32U4;

Sonar::Sonar(int pin) {
  _pin=pin;
}

float Sonar::readDist(){
  float duration, inches, cm;
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_pin, LOW);

  pinMode(_pin, INPUT);
  duration = pulseIn(_pin, HIGH);
  cm = duration/29.0/2.0;
  return cm;
}

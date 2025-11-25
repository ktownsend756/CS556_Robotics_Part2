#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;
#include "odometry.h"
#include "sonar.h"
#include "PIDcontroller.h"

LineSensors lineSensors;
Motors motors;
Servo servo;

Sonar sonar = sonar(4);

//Physical Robot Parameters 
#define diaL 3.2 
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio);
Encoders encoders;

//Odometry Variables
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = 10;
float y = 10;
float theta = 0;
float x_last = 0.0;
float y_last = 0.0;
float theta_last = 0.0;

//PID Controller Variables
#define kp 200
#define kd 5
#define ki 0.5
#define clamp_i 50
#define base_speed 100
#define minOutput -100
#define maxOutput 100

PIDcontroller PIDcontroller(kp, ki, kd, minOutput, maxOutput, clamp_i);
double PID_OUT_DISTANCE;

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

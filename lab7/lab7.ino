#include <Pololu3piPlus32U4.h>
#include "printOLED.h"
#include "odometry.h" //If using odometry, import odometry.h and odometry.cpp
#include "PIDcontroller.h" //Import your PIDcontroller.h and PIDcontroller.cpp from last lab then uncomment
using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;

//Odometry Parameters
#define diaL 3.2
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75
#define DEAD_RECKONING false

//Update kp, kd, and ki based on your testing (First PIDcontroller for angle)
#define minOutputAng -100
#define maxOutputAng 100
#define kpAng 100 //Tune Kp here
#define kdAng 20//Tune Kd here
#define kiAng 2 //Tune Ki here
#define clamp_iAng 15 //Tune ki integral clamp here
#define base_speedAng 50

//Update kp, kd, and ki based on your testing (Second PIDcontroller for velocity) (Task 2.3)
#define minOutputVel -50
#define maxOutputVel 150
#define kpVel 50 //Tune Kp here
#define kdVel 35 //Tune Kd here
#define kiVel 5 //Tune Ki here
#define clamp_iVel 20 //Tune ki integral clamp here
#define base_speedVel 50

Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING); //Uncomment if using odometry class
PIDcontroller pidcontroller(kpAng, kiAng, kdAng, minOutputAng, maxOutputAng, clamp_iAng); //Uncomment after you import PIDController
//Write your second PIDcontroller object here (Task 2.3)
PIDcontroller pidcontroller2(kpVel, kiVel, kdVel, minOutputVel, maxOutputVel, clamp_iVel);
//Feel free to use this in your PD/PID controller for target values
// Given goals in cm and radians
const float goal_x = 100;
const float goal_y = 100;
const float goal_theta = .785; // Must put in radians

//odometry
int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x, y, theta;

//Lab 7
//Note: Here are some suggested variables to use for your code.
double PIDout_theta, PIDout_distance; //Output variables for your controllers
double angle_to_goal, actual_angle; //Keeping track of angle
double dist_to_goal = 0.0; //Keeping track of robot's distance to goal location

void setup() {
  Serial.begin(9600);
}

void loop() {

  //Use this code if you are using odometry. Comment out if you are not.
  //If using, consider turning this into its own function for repeated use.
  // Read data from encoders
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();

  // Increment total encoder cound
  encCountsLeft += deltaL;
  encCountsRight += deltaR;  

  odometry.update_odom(encCountsLeft,encCountsRight, x, y, theta); //calculate robot's position


  //Lab 7
  //Note: To help with testing, print the theta and PID outputs to serial monitor.
  Serial.println(PIDout_theta);
  Serial.println(PIDout_distance);

  /*TASK 2.1 (DONE)
  Move your PIDController.h and PIDController.cpp files here to use for the following tasks.
  Also move your odometry.h and odometry.cpp if you decide to use it for 
  measuring the angle of your robot.
  
  Utilize your PIDController to go to a specific location.
  
  Hint: Utilize these functions to find your thetas
  angle_to_goal = atan2(?, ?);
  //atan2(sin(x),cos(x))=x on [-π, π) and not on [0,2π) 
  //=> we do this to make sure the range of actual_angle and goal_to_angle is the same
  actual_angle = atan2(?, ?);
  
  Write your code below and comment out when moving to the next task.*/

  /*
  angle_to_goal = atan2(goal_y - y, goal_x - x);
  actual_angle = atan2(sin(theta), cos(theta));
  PIDout_theta = pidcontroller.update(actual_angle, angle_to_goal);

  int leftspeed = base_speedAng - PIDout_theta;
  int rightspeed = base_speedAng + PIDout_theta;

  motors.setSpeeds(leftspeed, rightspeed);
  */

  /*TASK 2.2 (DONE)
  Improve the baseline solution by telling the robot to stop when it gets close 
  enough to the goal.
  Write your code below and comment out when moving to the next task.*/

  /*
  angle_to_goal = atan2(goal_y - y, goal_x - x);
  actual_angle = atan2(sin(theta), cos(theta));
  PIDout_theta = pidcontroller.update(actual_angle, angle_to_goal);

  int leftspeed = base_speedAng - PIDout_theta;
  int rightspeed = base_speedAng + PIDout_theta;

  dist_to_goal = sqrt(pow((goal_x - x), 2) + pow((goal_y - y), 2));
  if(dist_to_goal <= 10){ //checks if goal is less than 10cm away, and makes the robot halt
    motors.setSpeeds(0, 0);
  }
  else{ //Otherwise robot continues to move at the same speed towards the goal
    motors.setSpeeds(leftspeed, rightspeed);
  }
  */

  /*TASK 2.3 (DONE)
  Improve the solution further by using a second PID controller to control the velocity
  as it goes towards the goal.
  Write your code below.*/

  
  angle_to_goal = atan2(goal_y - y, goal_x - x);
  Serial.print("Angle to goal: ");
  Serial.println(angle_to_goal);
  actual_angle = atan2(sin(theta), cos(theta));
  Serial.print("Actual Angle: ");
  Serial.println(actual_angle);
  PIDout_theta = pidcontroller.update(actual_angle, angle_to_goal);

  
  dist_to_goal = sqrt(pow(x - goal_x, 2) + pow(y - goal_x, 2));
  Serial.print("Distance to Goal: ");
  Serial.println(dist_to_goal);
  
  //Track the distance between robot's current position and the goal and use output to control velocity
  PIDout_distance = pidcontroller2.update(0, dist_to_goal); 

  //PIDout_distance is the new velocity that replaces base_speed
  int leftspeed = int(PIDout_distance - PIDout_theta);
  int rightspeed = int(PIDout_distance + PIDout_theta);

  if(dist_to_goal <= 10){ //checks if goal is less than 10cm away, and makes the robot halt
    motors.setSpeeds(0, 0);
  }
  else{ //Otherwise robot continues to move at the same speed towards the goal
    motors.setSpeeds(leftspeed, rightspeed);
  }
  
}

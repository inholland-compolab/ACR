//Controlling arduino motors using ROS Twist keyboard
//rosrun rosserial_python serial_node.py /dev/ttyUSB0
//rosrun teleop_twist_keyboard teleop_twist_keyboard.py

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif
#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#define USE_USBCON

ros::NodeHandle  nh;

geometry_msgs::Twist msg;

float move1;
float move2;
int nSteppers = 4;

const int ena[] = {46, 47, 44, 45};
const int motionEna[] = {4, 5, 6, 7};
const int dir[] = {50, 51, 48, 49};

void callback(const geometry_msgs::Twist& cmd_vel)
{
  move1 = cmd_vel.linear.x;
  move2 = cmd_vel.angular.z;
  if (move1 < 0 && move2 == 0)
  {
    front();
  }
  else if (move1 == 0 && move2 < 0 )
  {
    left();
  }
  else if (move1 == 0 && move2 > 0 )
  {
    right();
  }
  else if (move1 > 0 && move2 == 0)
  {
    back();
  }
  else if (move1 < 0 && move2 > 0)
  {
    turnleft();
  }
    else if (move1 < 0 && move2 < 0)
  {
    turnright();
  }
  else
  {
    die();
  }
}

ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", callback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
for (int i = 0; i < nSteppers; i++) {
    pinMode(ena[i], OUTPUT);   
    pinMode(dir[i], OUTPUT);
    pinMode(motionEna[i], OUTPUT);

    digitalWrite(ena[i], LOW); //LOW == Active
  }

//    die();

    digitalWrite(dir[0], HIGH );
    digitalWrite(dir[1], LOW);
    digitalWrite(dir[2], HIGH);
    digitalWrite(dir[3], LOW);

}

void loop()
{   
  nh.spinOnce();
  delay(1);
}


void front()
{
for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i] , HIGH);
      }

      delay(10);

      digitalWrite(dir[0], LOW);
      digitalWrite(dir[1], HIGH);
      digitalWrite(dir[2], LOW);
      digitalWrite(dir[3], HIGH);

      for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i], LOW);
      }
}
void back()
{
for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i] , HIGH);
      }

      delay(10);

      digitalWrite(dir[0], HIGH);
      digitalWrite(dir[1], LOW);
      digitalWrite(dir[2], HIGH);
      digitalWrite(dir[3], LOW);

      for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i], LOW); 
      }
}
// move Left
void left()
{
    for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i] , HIGH);
      }

      delay(10);

      digitalWrite(dir[0], HIGH);
      digitalWrite(dir[1], HIGH);
      digitalWrite(dir[2], LOW);
      digitalWrite(dir[3], LOW);

      for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i], LOW);
      }

}
// move Right
void right()
{
for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i] , HIGH);
      }

      delay(10);

      digitalWrite(dir[0], LOW);
      digitalWrite(dir[1], LOW);
      digitalWrite(dir[2], HIGH);
      digitalWrite(dir[3], HIGH);

      for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i], LOW);
      } 
}
// Turn Left
void turnleft()
{
    for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i] , HIGH);
      }

      delay(10);

      digitalWrite(dir[0], LOW);
      digitalWrite(dir[1], LOW);
      digitalWrite(dir[2], LOW);
      digitalWrite(dir[3], LOW);

      for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i], LOW);
      }

}
// Turn Right
void turnright()
{
for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i] , HIGH);
      }

      delay(10);

      digitalWrite(dir[0], HIGH);
      digitalWrite(dir[1], HIGH);
      digitalWrite(dir[2], HIGH);
      digitalWrite(dir[3], HIGH);

      for (int i = 0; i < nSteppers; i++) {
        digitalWrite( motionEna[i], LOW);
      } 
}
void die()
{
    for (int i = 0; i < nSteppers; i++) {
      digitalWrite( motionEna[i] , HIGH);
    }
}

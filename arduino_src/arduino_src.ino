//Controlling arduino motors using ROS Twist keyboard
//rosrun rosserial_python serial_node.py /dev/ttyUSB0
//rosrun teleop_twist_keyboard teleop_twist_keyboard.py

#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define N_MOTORS 4
#define N_STEPS 5000

ros::NodeHandle  nh;
geometry_msgs::Twist msg;

float move1;
float move2;

const int ena[] = {46, 47, 44, 45};
const int otherEnable[] = {4, 5, 6, 7};
const int dir[] = {50, 51, 48, 49};
const int motionEna[] = {8, 9, 10, 11};

// TODO Use fast PWM mode on a 256 prescaler with the settings register
// 	'Toggle OCRnA on compare match' to generate a pwm signal at 50%
// 	duty cycle with a variable frequency (122 - 31250Hz)

void twist_callback(const geometry_msgs::Twist& cmd_vel)
{  
	move1 = cmd_vel.linear.x;
	move2 = cmd_vel.angular.z;
	
	if (move1 < 0 && move2 == 0) {
		front();
	}
	else if (move1 == 0 && move2 < 0 ) {
		left();
	}
	else if (move1 == 0 && move2 > 0 ) {
		right();
	}
	else if (move1 > 0 && move2 == 0) {
		back();
	}
	else if (move1 < 0 && move2 > 0) {
		turnleft();
	}
	else if (move1 < 0 && move2 < 0) {
		turnright();
	}
	else {
		die();
	}
}

ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", twist_callback);

void setup() {
	nh.initNode();
	nh.subscribe(sub);
	
	for (int i=0; i<N_MOTORS; i++) {
		pinMode(ena[i], OUTPUT);   
		pinMode(dir[i], OUTPUT);
		pinMode(motionEna[i], OUTPUT);
		pinMode(otherEnable[i], OUTPUT);
		
		digitalWrite(motionEna[i], LOW);
		digitalWrite(otherEnable[i], LOW);
		digitalWrite(dir[i], LOW);
		digitalWrite(ena[i], LOW); //LOW == Active
	} 
}

void loop() {   
	nh.spinOnce();
	delay(100);
}

void front() {        
	// Set wheel directions
	digitalWrite(dir[0], LOW);
	digitalWrite(dir[1], HIGH);
	digitalWrite(dir[2], LOW);
	digitalWrite(dir[3], HIGH);
	
	move();
}

void back() {
  // Set wheel directions
	digitalWrite(dir[0], HIGH);
	digitalWrite(dir[1], LOW);
	digitalWrite(dir[2], HIGH);
	digitalWrite(dir[3], LOW);

	move();
}

void left() {
  // Set wheel directions    
	digitalWrite(dir[0], HIGH);
	digitalWrite(dir[1], HIGH);
	digitalWrite(dir[2], LOW);
	digitalWrite(dir[3], LOW);

	move();
}

void right() {
  // Set wheel directions     
	digitalWrite(dir[0], LOW);
	digitalWrite(dir[1], LOW);
	digitalWrite(dir[2], HIGH);
	digitalWrite(dir[3], HIGH);
	
	move();
}

void turnleft() {
  // Set wheel directions
	digitalWrite(dir[0], LOW);
	digitalWrite(dir[1], LOW);
	digitalWrite(dir[2], LOW);
	digitalWrite(dir[3], LOW);

	move();
}

void turnright() {
  // Set wheel directions
	digitalWrite(dir[0], HIGH);
	digitalWrite(dir[1], HIGH);
	digitalWrite(dir[2], HIGH);
	digitalWrite(dir[3], HIGH);

	move();
}

void die() {
	for (int i=0; i<N_MOTORS; ++i) {
		digitalWrite(motionEna[i] , HIGH);
	}
}

void move() {
	for(int j=0; j<N_STEPS; ++j){
		for(int i=0; i<N_MOTORS; ++i) {
			digitalWrite(motionEna[i], HIGH);
		}

		delay(1);
    
		for(int i=0; i<N_MOTORS; ++i) {
			digitalWrite(motionEna[i], LOW);
		}
    		delay(1);
	}
}

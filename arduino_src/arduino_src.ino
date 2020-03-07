//Controlling arduino motors using ROS Twist keyboard rosrun rosserial_python serial_node.py /dev/ttyUSB0
//rosrun teleop_twist_keyboard teleop_twist_keyboard.py

#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define N_MOTORS 4

ros::NodeHandle  nh;
geometry_msgs::Twist msg;

float move1;
float move2;

const int ena[] = {46, 47, 44, 45};
const int otherEnable[] = {4, 5, 6, 7};
const int dir[] = {50, 51, 48, 49};
const int motionEna[] = {9, 10, 11, 12};

// Is called when a new twist message is published on the given topic
void twist_callback(const geometry_msgs::Twist& cmd_vel)
{  
	die();

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
	} else {
		return;
	}
	live();
}

// Is currently in PWM mode, just because it's easy to set up, varying
// the duty cycle will have no effect on the motors, the prescaler will
void motor_pwm_setup() {
	// Timer 1 PWM mode 64 prescaler
	TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
	TCCR1B = _BV(CS11);
	OCR1A = 180;
	OCR1B = 180;

	// Timer 2 PWM mode 64 prescaler
	TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
	TCCR2B = _BV(CS22);
	OCR2A = 180;
	OCR2B = 180;
}

ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", twist_callback);

void setup() {
	// Set up ROS node communication
	nh.initNode();
	nh.subscribe(sub);

	// Set up pins
	for (int i=0; i<N_MOTORS; i++) {
		pinMode(ena[i], OUTPUT);   
		pinMode(dir[i], OUTPUT);
		pinMode(motionEna[i], OUTPUT);
		pinMode(otherEnable[i], OUTPUT);
		
		digitalWrite(otherEnable[i], LOW);
		digitalWrite(dir[i], LOW);
		digitalWrite(ena[i], LOW); //LOW == Active
	} 
	die();                // Kill the motors
	motor_pwm_setup();    // and start the motor clock signals
}

void loop() {
	nh.spinOnce();    // Check ROS for any incoming messages
}

// Kill all motors
void die() {
	for (int i=0; i<N_MOTORS; ++i) {
		digitalWrite(motionEna[i] , HIGH);
	}
}

// Set all motors on
void live() {
	for (int i=0; i<N_MOTORS; ++i) {
		digitalWrite(motionEna[i] , LOW);
	}
}

// The following fucntions set set the stepper controllers
// to turn the motors in the given direction
// NOTE: left and right are strafing directions
void front() {        
	digitalWrite(dir[0], LOW);
	digitalWrite(dir[1], HIGH);
	digitalWrite(dir[2], LOW);
	digitalWrite(dir[3], HIGH);
}

void back() {
	digitalWrite(dir[0], HIGH);
	digitalWrite(dir[1], LOW);
	digitalWrite(dir[2], HIGH);
	digitalWrite(dir[3], LOW);
}

void left() {
	digitalWrite(dir[0], HIGH);
	digitalWrite(dir[1], HIGH);
	digitalWrite(dir[2], LOW);
	digitalWrite(dir[3], LOW);
}

void right() {
	digitalWrite(dir[0], LOW);
	digitalWrite(dir[1], LOW);
	digitalWrite(dir[2], HIGH);
	digitalWrite(dir[3], HIGH);
}

void turnleft() {
	digitalWrite(dir[0], LOW);
	digitalWrite(dir[1], LOW);
	digitalWrite(dir[2], LOW);
	digitalWrite(dir[3], LOW);
}

void turnright() {
	digitalWrite(dir[0], HIGH);
	digitalWrite(dir[1], HIGH);
	digitalWrite(dir[2], HIGH);
	digitalWrite(dir[3], HIGH);
}


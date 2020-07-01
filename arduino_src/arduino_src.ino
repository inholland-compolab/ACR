// Controlling arduino motors using ROS Twist keyboard 
// Run each of the following commands in a new command line
//
// rosrun teleop_twist_keyboard teleop_twist_keyboard.py
// rosrun rosserial_python serial_node.py </dev/ttyUSB[0-9] or /dev/ttyACM[0-9]>

#include <stdlib.h>
#include <ros.h>
#include <geometry_msgs/Twist.h> 

#define N_MOTORS 4
#define linear_factor 25
#define angular_factor 50

ros::NodeHandle nh;
geometry_msgs::Twist msg;

float x;
float y;
float z;

const int ena[] = {46, 47, 44, 45};
const int otherEnable[] = {4, 5, 6, 7};
const int dir[] = {50, 51, 48, 49};

// Is called when a new twist message is published on the given topic
void twist_callback(const geometry_msgs::Twist& cmd_vel)
{  
	die();

	x = cmd_vel.linear.x;
	y = cmd_vel.linear.y;
	z = cmd_vel.angular.z;

	if (z == 0) {
		if (x > 0 && y == 0) {
			front();	
		} else if (x < 0 && y == 0) {
			back();	
		} else if (x == 0 && y > 0) {
			left();
		} else if (x == 0 && y < 0) {
			right();
		} else {
			return;
	        }
                if (x == 0) {
                    live(linear_factor / abs(y)); 
                } else {
                    live(linear_factor / abs(x)); 
                }
	} else {
		if(z > 0) {
			turnright();
		} else {
			turnleft();
		}
		live(angular_factor / abs(z));
	}
}

// Is currently in PWM mode, just because it's easy to set up, varying
// the duty cycle will have no effect on the motors, the prescaler will
void motor_pwm_setup() {
	pinMode(11, OUTPUT);
	pinMode(12, OUTPUT);

	TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12);
	
	OCR1A = 500;
}

ros::Subscriber <geometry_msgs::Twist> sub("/cmd_vel", twist_callback);

void setup() {
	// Set up ROS node communication
	nh.initNode();
	nh.subscribe(sub);
	pinMode(13, OUTPUT);

	// Set up pins
	for (int i=0; i<N_MOTORS; i++) {
		pinMode(ena[i], OUTPUT);   
		pinMode(dir[i], OUTPUT);
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
		digitalWrite(otherEnable[i] , HIGH);
	}
	digitalWrite(13, LOW);
}

// Set all motors on
void live(float speed) {
	OCR1A = speed;

	for (int i=0; i<N_MOTORS; ++i) {
		digitalWrite(otherEnable[i] , LOW);
	}
	digitalWrite(13, HIGH);
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


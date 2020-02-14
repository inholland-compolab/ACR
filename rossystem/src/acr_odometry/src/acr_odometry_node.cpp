#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "acr_odometry_node.hpp"


OdometryNode::OdometryNode():
	odom_publisher_(nh_.advertise<nav_msgs::Odometry>("odometry", 100)),
	twist_sub_(nh_.subscribe("cmd_vel", 100, &OdometryNode::twistCallback, this)) 
{
	this->x = 0.0;
	this->y = 0.0;
	this->th = 0.0;

	ros::spin();
}

void OdometryNode::twistCallback(const geometry_msgs::Twist& msg) 
{
	ROS_INFO("I heard: [%f]", msg.linear.x);
	publishOdometry(msg);
}

void OdometryNode::publishOdometry(const geometry_msgs::Twist& msg)
{
	double vx = msg.linear.x;
	double vy = msg.linear.y;
	double vth = msg.angular.z;
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();
	
	ros::Rate r(10);
	
	while(ros::ok())
	{
		current_time = ros::Time::now();
		
		//compute odometry in a typical way given the velocities of the robot
		double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
		double delta_th = vth * dt;
		
		x += delta_x;
		y += delta_y;
		th += delta_th;
		
		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
		
		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";
		
		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;
		
		//send the transform
		odom_broadcaster_.sendTransform(odom_trans);
		
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		
		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;
		
		//publish the message
		odom_publisher_.publish(odom);
	
		last_time = current_time;
		ros::spinOnce();
		r.sleep();
	}
}



int main(int argc, char * argv[]) 
{
	ros::init(argc, argv, "odometry_publisher");
	OdometryNode node;	
}


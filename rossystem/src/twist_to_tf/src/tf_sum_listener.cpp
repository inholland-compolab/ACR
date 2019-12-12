#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_listener");
	
	ros::NodeHandle node;
	
	ros::Publisher pos_publisher = node.advertise<tf::StampedTransform>("pos", 10);
	
	tf::TransformListener listener;
	
	ros::Rate rate(10.0);
	
	while (node.ok()){
	  	tf::StampedTransform transform;
	  	try {
	    		listener.lookupTransform("/base_link", "/odom", ros::Time(0), transform);
	      } catch (tf::TransformException &ex) {
	      		ROS_ERROR("%s",ex.what());
	      		ros::Duration(1.0).sleep();
	    		continue;
		}
	      	tf::Transform pos;
	     
		// TODO not possible to just add them up?
	      	pos.setOrigin(pos.getOrigin() + transform.getOrigin());
	      	pos.setRotation(pos.getRotation() + transform.getRotation());
	      
	      	pos_publisher.publish(
			tf::StampedTransform(pos, ros::Time::now(), "map", "odom")
		);
	      	rate.sleep();
	}
	return 0;
};

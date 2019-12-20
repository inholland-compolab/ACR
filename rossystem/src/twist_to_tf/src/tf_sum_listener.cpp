#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>


int main(int argc, char** argv){
	ros::init(argc, argv, "my_tf_listener");
	
	ros::NodeHandle node;
	ros::Publisher pos_publisher = node.advertise<geometry_msgs::PoseStamped>("pos", 10);
	
	geometry_msgs::PoseStamped pose_msg = geometry_msgs::PoseStamped();
	tf::TransformListener listener;
	// tf::Transform tf_pos = tf::Transform();;
	

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
		// TODO not possible to just add them up?
		// tf_pos.setOrigin(tf_pos.getOrigin() + transform.getOrigin());
		// tf_pos.setRotation(tf_pos.getRotation() * transform.getRotation());
		
		// tf::StampedTransform(tf_pos, ros::Time::now(), "base_link", "odom");
		// geometry_msgs::TransformStamped geometry_pos;	
		// tf::transformStampedTFToMsg(tf_pos, geometry_pos);
	
	
		pose_msg.pose.position.x += transform.getOrigin().x();
		pose_msg.pose.position.y += transform.getOrigin().y();
		pose_msg.pose.position.z += transform.getOrigin().z();
		
		pose_msg.pose.orientation.z = transform.getRotation().z();
		
		// Generate Header
		pose_msg.header.stamp = ros::Time().now();
		pose_msg.header.frame_id = "pos";
			
		// Publish rviz-readable msg with header
	
		pos_publisher.publish(pose_msg);
	      	rate.sleep();
	}
	return 0;
};

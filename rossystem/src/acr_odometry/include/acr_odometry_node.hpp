#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdometryNode
{

public:
	OdometryNode();
	void twistCallback(const geometry_msgs::Twist& msg);
	void publishOdometry(const geometry_msgs::Twist& msg);
	
	double x;
	double y;
	double th; 

private:
	ros::NodeHandle nh_;
	ros::Subscriber twist_sub_;
	ros::Publisher odom_publisher_;

	tf::TransformBroadcaster odom_broadcaster_;
};


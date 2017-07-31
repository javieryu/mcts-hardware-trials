#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;

void odomCB(const nav_msgs::Odometry::ConstPtr& msg){
	static tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	
	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = "world";
	transformStamped.child_frame_id = "quad";
	
	transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = -1*msg->pose.pose.position.y;
    transformStamped.transform.translation.z = msg->pose.pose.position.z;
    
	transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
	transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
	transformStamped.transform.rotation.z = -1*msg->pose.pose.orientation.z;
	transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
	
	//~ Negations are to match the odometry with tf's natural xyz orientations
	
	br.sendTransform(transformStamped);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "odom_tf_node");
	
	ros::NodeHandle nh;
	
	ros::Subscriber odomSub = nh.subscribe("dji_sdk/odometry", 1000, &odomCB);
	
	ros::spin();
	
	return 0;
}



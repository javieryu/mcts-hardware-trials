#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>


using namespace std;

class LocalPlannerNode {
	
public:
	LocalPlannerNode() {
		ros::NodeHandle nh;
		grid_sub_ = nh.subscribe("rtabmap/grid_map", 1000, &gridCB);
		goal_pub_ = nh.advertise<geometry_msgs::Pose>("local_planner_goal",1000);
		
	}

	void gridCB(const nav_msgs::OccupancyGrid::ConstPtr& msg){
		geometry_msgs::Pose goal;
		
		
		
		goal_pub_.publish(goal);
	}
	
	
	
private:
	ros::Subscriber grid_sub_;
	ros::Publisher goal_pub_;
	
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "local_planner_node");
	
	LocalPlannerNode lp;
	
	ros::spin();
	
	return 0;
}

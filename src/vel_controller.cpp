#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <stdlib.h>
#include <custom_messages/DJI_Bridge_Travel_Speed_MSG.h>


using namespace std;

class ControllerNode {
	
public:
	ControllerNode() {
		ros::NodeHandle nh;
		depth_image_sub_ = nh.subscribe("zed/depth/depth_registered", 1000, &ControllerNode::depth_imageCB,this);
		speed_pub_ = nh.advertise<custom_messages::DJI_Bridge_Travel_Speed_MSG>("proportional_vel",1000);
		
		emergency_stop_ = false;
		nominal_vel_ = 5.0;
		depth_based_vel_ = nominal_vel_;
		
		ROS_INFO("Init DONE");
		
	}

	void depth_imageCB(const sensor_msgs::ImageConstPtr& msg){
		
		cv_bridge::CvImagePtr cv_ptr;
		
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
			//~ cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		
		// DEBUG CODE
		//~ double min,max;
		//~ cv::minMaxLoc(cv_ptr->image,&min,&max);
		//~ ROS_INFO("Min: %f, Max %f", min,max);
		
		
		cv::Mat blur_im;
		cv::GaussianBlur(cv_ptr->image, blur_im, cv::Size(21,21), 0, 0);
		
		
		// Unit depends on ZED launch file parameter "openni_depth_mode" 0 is meters, 1 is mm
		// Using depth mode 1:
		double lowerb = 3000;
		double upperb = 6000;
		
		cv::Mat bin_em;
		cv::Mat bin_prop;
		
		
		cv::inRange(blur_im,lowerb,upperb,bin_prop);
		cv::inRange(blur_im,400,lowerb,bin_em);
		
		vector<vector<cv::Point> > contours_prop;
		vector<cv::Vec4i> hierarchy_prop;
		cv::findContours(bin_prop.clone(), contours_prop, hierarchy_prop, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

		vector<vector<cv::Point> > contours_em;
		vector<cv::Vec4i> hierarchy_em;
		cv::findContours(bin_em.clone(), contours_em, hierarchy_em, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


		// First, check that there are no obstacles in the emergency stop range 0 to lowerb
		if (contours_em.size() > 0) {
			
			for ( int i = 0; i < contours_em.size(); i++){
					
					double a = cv::contourArea(contours_em[i]);
					
					if (a > 200.0){
						emergency_stop_ = true;
					}
					else{
						emergency_stop_=false;
					}	
			}
		}
		
		// Next, check for countours of size in the proportional zone
		// If there are any, find the average values the contour(s) and use the closest
		// to set the proportional velocity.
		
		double lowest_avg = upperb;		
		
		if (contours_prop.size() > 0) {
			
			for ( int i = 0; i < contours_prop.size(); i++){
					
					double a = cv::contourArea(contours_prop[i]);
					
					
					if (a > 1000.0){
						
						cv::Mat blob(blur_im.size(),CV_8UC1);
						cv::drawContours(blob, contours_prop, i, cv::Scalar(255),-1);
						
						cv::Scalar mean = cv::mean(blur_im,blob);
						
						
						if (mean[0] < lowest_avg) {
							lowest_avg = mean[0];
						} 
						
					}		
			}
			
			if(lowest_avg > lowerb){
				depth_based_vel_ = ((lowest_avg-lowerb)/(upperb-lowerb))*nominal_vel_;
			}
			else {
				depth_based_vel_ = 0;
			}
		
		}
		
		custom_messages::DJI_Bridge_Travel_Speed_MSG vel;
		vel.stop = emergency_stop_;
		vel.travel_speed = depth_based_vel_;
		
		
		speed_pub_.publish(vel);
		
		//~ ROS_INFO("Prop Vel: %f, Emergency Stop: %d", depth_based_vel_, emergency_stop_);
		
		//~ cv::imshow("prop_im", bin_prop);
		//~ cv::imshow("em_im", bin_em);
		//~ cv::waitKey(1);

	}
	
private:
	ros::Subscriber depth_image_sub_;
	ros::Publisher speed_pub_;
	
	bool emergency_stop_;
	double nominal_vel_;
	double depth_based_vel_;
	
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "vel_controller_node");
	
	ControllerNode c;
	
	ros::spin();
	
	return 0;
}

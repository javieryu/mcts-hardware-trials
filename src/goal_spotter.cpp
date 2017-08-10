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


using namespace std;

class GoalSpotterNode {
	
public:
	GoalSpotterNode() {
		ros::NodeHandle nh;
		image_sub_ = nh.subscribe("zed/rgb/image_rect_color", 1000, &GoalSpotterNode::imageCB,this);
		flag_pub_ = nh.advertise<std_msgs::Bool>("goal_flag",1000);
		
	}

	void imageCB(const sensor_msgs::ImageConstPtr& msg){
		
		std_msgs::Bool flag;
		flag.data = false;
		
		cv_bridge::CvImagePtr cv_ptr;
		
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//blur input
		cv::Mat blur_im;
		cv::GaussianBlur(cv_ptr->image,blur_im,cv::Size(5,5),0);
		
		
		// Convert the image from BGR to HSV
		cv::Mat hsv_im;
		cv::cvtColor(blur_im, hsv_im, cv::COLOR_BGR2HSV);
		
		// Find the pixels in the image within range
		// Use filter_gui.py to determine the correct HSV color ranges
		// TODO: Need two inRange?? Red color wrap around in HSV spectrum
		cv::Mat thresh_im;
		cv::inRange(hsv_im,cv::Scalar(0,190,210),cv::Scalar(20,255,255),thresh_im);
		
		
		vector<vector<cv::Point> > contours;
		vector<cv::Vec4i> hierarchy;
		cv::findContours(thresh_im.clone(), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


		
		if (contours.size() > 0) {
		
			for ( int i = 0; i < contours.size(); i++){
					double a = cv::contourArea(contours[i]);
					
					if (a > 250.0){
						flag.data = true;
					}	
			}
			
			// For tuning
			//~ ROS_INFO("Largest Contour Area: %f", largest_area);
			
			//~ if (largest_area > 250.0) {
				//~ flag.data = true;
			//~ }
		}
		
		flag_pub_.publish(flag);
		// DEBUG code
		//~ cv::imshow("thresholded",thresh_im);
		//~ cv::imshow("rgb", blur_im);
		//~ cv::waitKey(1);
	}
	
	
	
private:
	ros::Subscriber image_sub_;
	ros::Publisher flag_pub_;
	
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "goal_spotter_node");
	
	GoalSpotterNode gs;
	
	ros::spin();
	
	return 0;
}

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
		flag_pub_ = nh.advertise<std_msgs::Bool>("goal_spotter_node",1000);
		
	}

	void imageCB(const sensor_msgs::Image::ConstPtr& msg){
		
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
		cv::Mat thresh_im;
		cv::inRange(hsv_im,cv::Scalar(1,100,125),cv::Scalar(20,255,255),thresh_im);
		
		
		vector<vector<cv::Point> > contours;
		vector<cv::Vec4i> hierarchy;
		cv::findContours(thresh_im, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));


		
		if (contours.size() > 0) {
		
			double largest_area = 0;
			int largest_contour_index = -1;
		
			for ( int i = 0; i < contours.size(); i++){
					double a = cv::contourArea(contours[i]);
					
					if (a > largest_area){
						largest_area = a;
						largest_contour_index = i;
					}	
			}
			
			if (largest_area > 10.0) {
				flag.data = true;
			}
		}
		
		
		
		flag_pub_.publish(flag);
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

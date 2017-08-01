#!/usr/bin/env python2
import roslib
# loab.manifest('kin_')
import rospy
import sys

import numpy as np

# :load.manifest('kinect_hsv_gui')
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16MultiArray
			
hsv1 = [0,179,0,255,0,255]

def nothing(i):
	pass

	
def callback(img_sub):
		global hsv1
		
		bridge = CvBridge()
		
		try:
			cv_image = bridge.imgmsg_to_cv2(img_sub,'bgr8')
		except CvBridgeError as e:
			print(e)
		
		cv2.namedWindow('gui',0)
		cv2.createTrackbar('h_high','gui',hsv1[0],179,nothing)
		cv2.createTrackbar('h_low','gui',hsv1[1],179,nothing)
		cv2.createTrackbar('s_high','gui',hsv1[2],255,nothing)
		cv2.createTrackbar('s_low','gui',hsv1[3],255,nothing)
		cv2.createTrackbar('v_high','gui',hsv1[4],255,nothing)
		cv2.createTrackbar('v_low','gui',hsv1[5],255,nothing)
		
		hsv1[0]=cv2.getTrackbarPos('h_high','gui')
		hsv1[1]=cv2.getTrackbarPos('h_low','gui')
		hsv1[2]=cv2.getTrackbarPos('s_high','gui')
		hsv1[3]=cv2.getTrackbarPos('s_low','gui')
		hsv1[4]=cv2.getTrackbarPos('v_high','gui')
		hsv1[5]=cv2.getTrackbarPos('v_low','gui')
		
		frame_to_thresh = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		thresh = cv2.inRange(frame_to_thresh,(hsv1[0],hsv1[2],hsv1[4]),(hsv1[1],hsv1[3],hsv1[5]))
		
		cv2.imshow('actual',cv_image)
		cv2.imshow('filtered',thresh)
		cv2.waitKey(1)
		
	


def main(args):
	rospy.init_node('hsv_gui', anonymous='True')
	
	
	
	
	img1_sub = rospy.Subscriber('/zed/rgb/image_rect_color', Image, callback)
	
	try:
		rospy.spin();
	except(KeyboardInterrupt):	
		cv2.destroyAllWindows()


if __name__=='__main__':
	main(sys.argv)
	

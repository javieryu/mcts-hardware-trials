#!/usr/bin/env python2
import roslib
# loab.manifest('kin_')
import rospy
import sys

import numpy as np

# :load.manifest("kinect_hsv_gui")
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16MultiArray

class filter_gui:

	def __init__( self ):
		self.hsv1_pub = rospy.Publisher("hsv_code",Int16MultiArray,queue_size=10)
		
		self.bridge = CvBridge()
		
		self.img1_sub = rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.callback)
		self.hsv1 = [0,179,0,255,0,255,1]

	def nothing(i,j):
		print("nothing called")	
		pass

	def callback(self,img_sub):
		
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img_sub,"bgr8")
		except CvBridgeError as e:
			print(e)
		
		
		
		frame_to_thresh = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		
		cv2.namedWindow("gui")
		cv2.createTrackbar('h_high','gui',self.hsv1[0],179,self.nothing)
		cv2.createTrackbar("h_low","gui",self.hsv1[1],179,self.nothing)
		cv2.createTrackbar("s_high","gui",self.hsv1[2],255,self.nothing)
		cv2.createTrackbar("s_low","gui",self.hsv1[3],255,self.nothing)
		cv2.createTrackbar("v_high","gui",self.hsv1[4],255,self.nothing)
		cv2.createTrackbar("v_low","gui",self.hsv1[5],255,self.nothing)
		cv2.createTrackbar("kinect #","gui", self.hsv1[6],3,self.nothing)

		self.hsv1[0]=cv2.getTrackbarPos("h_high","gui");
		self.hsv1[1]=cv2.getTrackbarPos("h_low","gui");
		self.hsv1[2]=cv2.getTrackbarPos("s_high","gui");
		self.hsv1[3]=cv2.getTrackbarPos("s_low","gui");
		self.hsv1[4]=cv2.getTrackbarPos("v_high","gui");
		self.hsv1[5]=cv2.getTrackbarPos("v_low","gui");
		
		self.hsv1[6]=cv2.getTrackbarPos("kinect #","gui");
		

		#msg = Int16MultiArray()
		#msg.data = self.hsv1;
		#msg = [h_min,h_max,s_min,s_max,v_min,v_max]
		#self.hsv1_pub.publish(msg)

		thresh = cv2.inRange(frame_to_thresh,(self.hsv1[0],self.hsv1[2],self.hsv1[4]),(self.hsv1[1],self.hsv1[3],self.hsv1[5]))
		
		#if self.hsv1[6]==self.id_to_num(img_sub.header.frame_id):
		cv2.imshow('actual',cv_image)
		cv2.imshow("filtered",thresh)
		cv2.waitKey(3)

	
	def id_to_num(_,frame_id):
		if frame_id == "camera_rgb_optical_frame":
			return 1
		if frame_id == "kinect2":
			return 2
		if frame_id == "kinect3":
			return 3
		print("here")
		return 0

def main(args):
	ic = filter_gui()
	rospy.init_node("hsv_gui", anonymous="True")
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("exiting")
	cv2.destroyAllWindows()


if __name__=='__main__':
	main(sys.argv)
	

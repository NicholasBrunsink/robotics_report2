#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Code adapted from flip_image.py and lecture code
# help also recieved from docs.opencv.org for circle detection

img_received = False
# define a 720x1280 3-channel image with all pixels equal to zero
rgb_img = np.zeros((720, 1280, 3), dtype = "uint8")


# get the image message
def get_image(ros_img):
	global rgb_img
	global img_received
	# convert to opencv image
	rgb_img = CvBridge().imgmsg_to_cv2(ros_img, "rgb8")
	# raise flag
	img_received = True

	
def main():
	# define the node, subcribers, and publishers
	rospy.init_node('detect_ball', anonymous = True)
	img_sub = rospy.Subscriber("/camera/color/image_raw", Image, get_image) 
	img_pub = rospy.Publisher('/ball_2D', Image, queue_size = 1)
	
	# create a focus window to isolate ball movement from background
	crop_img = np.zeros((rgb_img.shape[0], rgb_img.shape[1], 1), dtype = "uint8")
	crop_img = cv2.rectangle(crop_img, (120, 120), (crop_img.shape[1]-120, crop_img.shape[0]-120), 255, -1)
	
	# set the loop frequency
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# make sure we process if the camera has started streaming images
		if img_received:
			hsv = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
			# define the upper and lower ranges
			lower_yellow_hsv = np.array([20,5,1])
			upper_yellow_hsv = np.array([60,255,255])
			# filter the image 
			yellow_mask = cv2.inRange(hsv, lower_yellow_hsv, upper_yellow_hsv)
			# crop image to inner defined area to remove background objects
			cropped_hsv = cv2.bitwise_and(yellow_mask, yellow_mask, mask=crop_img)
			
			# detect circles in img using Hough Circle detection algorithm
			circles = cv2.HoughCircles(cropped_hsv, cv2.HOUGH_GRADIENT, 1, cropped_hsv.shape[0]/8, param1=40, param2=10, minRadius=50, maxRadius=80)
			# create empty img to store circle in clean img
			circ_detect = np.zeros((rgb_img.shape[0], rgb_img.shape[1], 1), dtype = "uint8")
			# draw largest detected circle in empty img
			if circles is not None:
				ball = circles[0][0]
				ball = np.uint16(np.around(ball)) 
				cv2.circle(circ_detect, (ball[0], ball[1]), 1, 255, ball[2]*2)
				
				# draw circle on top of ball in original image
				cv2.circle(cropped_hsv, (ball[0], ball[1]), 1, 255, ball[2]*2)
			
			# convert cropped img to ros msg and publish it
			img_msg = CvBridge().cv2_to_imgmsg(cropped_hsv, encoding="mono8")
			# publish final cropped img
			img_pub.publish(img_msg)
			
		# pause until the next iteration			
		rate.sleep()
main()

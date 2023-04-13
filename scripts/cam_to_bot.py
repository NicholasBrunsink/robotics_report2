#!/usr/bin/env python3

# importing dependancies
import rospy
from std_msgs.msg import Bool
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import XYZarray, SphereParams
import tf2_ros
from tf.transformations import *
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
import math

# Twist to store initial tool pose
toolpose = Twist()
# SphereParams to store initial sphere parameters
sphere = SphereParams()

# initializing flag for determining when initialization is enabled
toggleInit = False

def twist_callback(data):
	'''
	called when /ur5e/toolpose is published to store initial toolpose data
	'''
	toolpose.linear.x = data.linear.x
	toolpose.linear.y = data.linear.y
	toolpose.linear.z = data.linear.z
	toolpose.angular.x = data.angular.x
	toolpose.angular.y = data.angular.y
	toolpose.angular.z = data.angular.z
	
	
def toggle_callback(data):
	'''
	called when /toggleInit is published for initialization
	'''
	global toggleInit
	toggleInit = data.data
	
def sphere_callback(data):
	'''
	called when /sphere_params is published to store sphere data
	'''
	global sphere
	sphere.xc = data.xc
	sphere.yc = data.yc
	sphere.zc = data.zc
	sphere.radius = data.radius

def main():
	# initialize node
	rospy.init_node('cam_to_bot', anonymous=True)
	
	# initialize Buffer and Listener for Transform conversion 
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	
	# publisher for sphere in base frame
	sphere_pub = rospy.Publisher('/convert/sphereCoord', SphereParams, queue_size = 10)
	# publisher for initial conditions
	init_pub = rospy.Publisher('/convert/initCoord', Twist, queue_size=10)
	# subscriber for toggling movement
	movebool_sub = rospy.Subscriber('/toggleInit', Bool, toggle_callback)
	
	while not rospy.is_shutdown():
		# initialize SphereParams for publishing
		exportSphere = SphereParams()
		# subscriber for initial position and rotation
		toolpose_sub = rospy.Subscriber('/ur5e/toolpose', Twist, twist_callback)
		# subscriber for sphere in camera frame
		sphereparam_sub = rospy.Subscriber('/sphere_params', SphereParams, sphere_callback)
	
		# hold execution until initialization is started
		print("Waiting for initialization msg on /toggleInit")
		while not rospy.is_shutdown() and not toggleInit:
			pass
		
		# unregister from /ur5e/toolpose and /sphere_params to prevent mid execution changes
		toolpose_sub.unregister()
		sphereparam_sub.unregister()
		
		# set loop rate to 10 Hz
		loop_rate = rospy.Rate(10)
		
		print("Initializing")
		while not rospy.is_shutdown():
			if toggleInit:
				# try getting the most update transformation between the tool frame and the base frame
				try:
					trans = tfBuffer.lookup_transform("base", "camera_color_optical_frame", rospy.Time())
				except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
					print('Frames not available!!!')
					loop_rate.sleep()
					continue
				
				# create a PointStamped based on the sphere location in the camera frame
				pt_in_cam = tf2_geometry_msgs.PointStamped()
				pt_in_cam.header.frame_id = 'camera_color_optical_frame'
				pt_in_cam.header.stamp = rospy.get_rostime()
				pt_in_cam.point.x = sphere.xc
				pt_in_cam.point.y = sphere.yc
				pt_in_cam.point.z = sphere.zc
				# convert the 3D point to the base frame coordinates
				pt_in_base = tfBuffer.transform(pt_in_cam,'base', rospy.Duration(1.0))
				
				# set new SphereParams based on base frame 
				exportSphere.xc = pt_in_base.point.x
				exportSphere.yc = pt_in_base.point.y
				exportSphere.zc = pt_in_base.point.z
				exportSphere.radius = sphere.radius
					
				print("\nSphere params")
				print(exportSphere)
				print("\nToolpose")
				print(toolpose)
				
			else:
				print("\nInitialized Sphere")
				print(exportSphere)
				print("\nInitialized Toolpoase")
				print(toolpose)
			
			# publishing exportSphere and toolpose for pickup_motion.py
			sphere_pub.publish(exportSphere)
			init_pub.publish(toolpose)
			
			# wait for 0.1 seconds until the next loop and repeat
			loop_rate.sleep()
main()

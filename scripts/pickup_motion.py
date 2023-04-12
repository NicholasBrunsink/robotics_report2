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

# initializing Twist to store initial tool pose
toolpose = Twist()
sphere = SphereParams()
# initializing flag for determining when toolpose data is recieved
twistDataRecieved = False
sphereDataRecieved = False
holdMovement = True

def twist_callback(data):
	'''
	called when /ur5e/toolpose is published to store initial toolpose data
	'''
	global twistDataRecieved
	if not twistDataRecieved:
		toolpose.linear.x = data.linear.x
		toolpose.linear.x = data.linear.y
		toolpose.linear.z = data.linear.z
		toolpose.angular.x = data.angular.x
		toolpose.angular.y = data.angular.y
		toolpose.angular.z = data.angular.z
		twistDataRecieved = True
	
def toggle_callback(data):
	global holdMovement
	holdMovement = data.data
	
def sphere_callback(data):
	global sphere
	sphere.xc = data.xc
	sphere.yc = data.yc
	sphere.zc = data.zc
	sphere.radius = data.radius
	if sphere.radius > 0.0005:
		sphereDataRecieved = True

def main():
	# initialize node
	rospy.init_node('pickup_object', anonymous=True)
	tfBuffer = tf2_ros.Buffer()
	
	# create publisher to publish Plan 
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# create subscriber to grab initial position and rotation
	toolpose_sub = rospy.Subscriber('/ur5e/toolpose', Twist, twist_callback)
	movebool_sub = rospy.Subscriber('/convert/initCoord', Bool, toggle_callback)
	
	sphereparam_sub = rospy.Subscriber('/convert/sphereCoord', SphereParams, sphere_callback)
	
	
	

	# hold execution until initial position and rotation is set
	global twistDataRecieved
	global sphereDataRecieved
	while not twistDataRecieved or holdMovement:
		print()
		print(twistDataRecieved)
		print(sphere)
		pass
	# unsubscribe from /ur5e/toolpose after initial pos is stored
	toolpose_sub.unregister()
	# unsubscribe from /sphereparam after storing good sphere location
	sphereparam_sub.unregister()

	# set loop rate to 10 Hz
	loop_rate = rospy.Rate(10)
	
	# define a plan variable
	plan = Plan() 
	
	plan_point1 = Twist()
	# define a point at the initial position
	plan_point1.linear.x = -0.7
	plan_point1.linear.y = -0.133
	plan_point1.linear.z = 0.43
	plan_point1.angular.x = toolpose.angular.x
	plan_point1.angular.y = toolpose.angular.y
	plan_point1.angular.z = toolpose.angular.z
	# add this point to the plan
	plan.points.append(plan_point1)
	
	plan_point2 = Twist()
	# define a point  below initial position
	plan_point2.linear.x = -0.7
	plan_point2.linear.y = -0.133
	plan_point2.linear.z = 0.1
	plan_point2.angular.x = toolpose.angular.x
	plan_point2.angular.y = toolpose.angular.y
	plan_point2.angular.z = toolpose.angular.z
	# add this point to the plan
	plan.points.append(plan_point2)

	plan_point3 = Twist()
	# Define a point above the final position
	plan_point3.linear.x = -0.134
	plan_point3.linear.y = -0.7
	plan_point3.linear.z = 0.43
	plan_point3.angular.x = toolpose.angular.x
	plan_point3.angular.y = toolpose.angular.y
	plan_point3.angular.z = toolpose.angular.z
	# add this point to the plan
	plan.points.append(plan_point3)
	
	plan_point4 = Twist()
	# define point away from initial position to be the final point
	plan_point4.linear.x = -0.134
	plan_point4.linear.y = -0.7
	plan_point4.linear.z = 0.1
	plan_point4.angular.x = toolpose.angular.x
	plan_point4.angular.y = toolpose.angular.y
	plan_point4.angular.z = toolpose.angular.z
	# add this point to the plan
	plan.points.append(plan_point4)
	
	while not rospy.is_shutdown():
		# publish the plan
		plan_pub.publish(plan)
		# wait for 0.1 seconds until the next loop and repeat
		loop_rate.sleep()
main()

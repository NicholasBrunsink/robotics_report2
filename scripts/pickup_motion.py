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
drop_point = Twist()

# initializing flag for determining when movement is enabled
toggleMove = False

# called when /convert/initCoord subscriber recieves new data
def twist_callback(data):
	toolpose.linear.x = data.linear.x
	toolpose.linear.y = data.linear.y
	toolpose.linear.z = data.linear.z
	toolpose.angular.x = data.angular.x
	toolpose.angular.y = data.angular.y
	toolpose.angular.z = data.angular.z
	
# called when /toggleMove subscriber recieves new data
def toggle_callback(data):
	global toggleMove
	toggleMove = data.data

# called when /convert/sphereCoord published new data
def sphere_callback(data):
	global sphere
	sphere.xc = data.xc
	sphere.yc = data.yc
	sphere.zc = data.zc
	sphere.radius = data.radius

def main():
	# initialize node
	rospy.init_node('pickup_object', anonymous=True)
	
	# create publisher to publish Plan 
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# subscriber for listening when to enable movement
	movebool_sub = rospy.Subscriber('/toggleMove', Bool, toggle_callback)
	
	while not rospy.is_shutdown():
		# create subscriber to grab initial position and rotation
		toolpose_sub = rospy.Subscriber('/convert/initCoord', Twist, twist_callback)
		sphereparam_sub = rospy.Subscriber('/convert/sphereCoord', SphereParams, sphere_callback)
	
		print("\nWaiting for move initialization on topic /toggleMove")
		# hold execution until initial position and rotation is set
		while not rospy.is_shutdown() and not toggleMove:
			pass
		
		print("Movement Initialized")
	
		# creating location to drop ball based on initial position
		drop_point.linear.x = toolpose.linear.x + 0.001
		drop_point.linear.y = toolpose.linear.y + 0.001
		drop_point.linear.z = sphere.zc + 0.5*sphere.radius
	
		# set loop rate to 10 Hz
		loop_rate = rospy.Rate(10)
	
		# define a plan variable
		plan = Plan() 
		
		plan_point1 = Twist()
		# define point away from initial position to be the final point
		plan_point1.linear.x = drop_point.linear.x
		plan_point1.linear.y = drop_point.linear.y
		plan_point1.linear.z = drop_point.linear.z + 0.05
		plan_point1.angular.x = toolpose.angular.x
		plan_point1.angular.y = toolpose.angular.y
		plan_point1.angular.z = toolpose.angular.z
		# add this point to the plan
		plan.points.append(plan_point1)
		
		plan_point2 = Twist()
		# define a point at the initial position
		plan_point2.linear.x = sphere.xc
		plan_point2.linear.y = sphere.yc
		plan_point2.linear.z = sphere.zc + 0.3
		plan_point2.angular.x = toolpose.angular.x
		plan_point2.angular.y = toolpose.angular.y
		plan_point2.angular.z = toolpose.angular.z
		# add this point to the plan
		plan.points.append(plan_point2)
		
		plan_point3 = Twist()
		# define a point  below initial position
		plan_point3.linear.x = sphere.xc
		plan_point3.linear.y = sphere.yc
		plan_point3.linear.z = sphere.zc + 0.07
		plan_point3.angular.x = toolpose.angular.x
		plan_point3.angular.y = toolpose.angular.y
		plan_point3.angular.z = toolpose.angular.z
		# add this point to the plan
		plan.points.append(plan_point3)
		
		plan_point4 = Twist()
		# Define a point above the final position
		plan_point4.linear.x = drop_point.linear.x
		plan_point4.linear.y = drop_point.linear.y
		plan_point4.linear.z = drop_point.linear.z + 0.2
		plan_point4.angular.x = toolpose.angular.x
		plan_point4.angular.y = toolpose.angular.y
		plan_point4.angular.z = toolpose.angular.z
		# add this point to the plan
		plan.points.append(plan_point4)
		
	
		while not rospy.is_shutdown() and toggleMove:
			# publish the plan
			plan_pub.publish(plan)
			# wait for 0.1 seconds until the next loop and repeat
			loop_rate.sleep()
main()

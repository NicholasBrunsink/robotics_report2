#!/usr/bin/env python3

# importing dependancies
import rospy
from std_msgs.msg import Bool
from ur5e_control.msg import Plan
from geometry_msgs.msg import Twist
from robot_vision_lectures.msg import XYZarray, SphereParams
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
from std_msgs.msg import UInt8

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
	
def createPoint(plan, x, y, z, roll, pitch, yaw, mode):
	plan_point = Twist()
	point_mode = UInt8()
	# define point away from initial position to be the final point
	plan_point.linear.x = x
	plan_point.linear.y = y
	plan_point.linear.z = z
	plan_point.angular.x = roll
	plan_point.angular.y = pitch
	plan_point.angular.z = yaw
	point_mode.data = mode
	plan.points.append(plan_point)
	plan.modes.append(point_mode)
	return plan

def main():
	# initialize node
	rospy.init_node('pickup_object', anonymous=True)
	
	# create publisher to publish Plan 
	plan_pub = rospy.Publisher('/plan', Plan, queue_size = 10)
	# subscriber for listening when to enable movement
	movebool_sub = rospy.Subscriber('/toggleMove', Bool, toggle_callback)
	# create subscriber to grab initial position and rotation
	toolpose_sub = rospy.Subscriber('/convert/initCoord', Twist, twist_callback)
	sphereparam_sub = rospy.Subscriber('/convert/sphereCoord', SphereParams, sphere_callback)
	
	
	# set loop rate to 10 Hz
	loop_rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		if not toggleMove:
			plan = Plan()
			# initial point close to initial position
			plan = createPoint(plan, toolpose.linear.x+0.001, toolpose.linear.y+0.001, toolpose.linear.z+0.001, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 0) 		
			# above ball
			plan = createPoint(plan, sphere.xc, sphere.yc, toolpose.linear.z, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 0)
			# grabbing ball
			plan = createPoint(plan, sphere.xc+0.001, sphere.yc+0.001, sphere.zc+0.001, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 0)
			plan = createPoint(plan, sphere.xc, sphere.yc, sphere.zc, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 2)
			plan = createPoint(plan, sphere.xc+0.001, sphere.yc+0.001, sphere.zc+0.001, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 0)
			# above ball
			plan = createPoint(plan, sphere.xc, sphere.yc, toolpose.linear.z, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 0)
			# move above drop position
			plan = createPoint(plan, sphere.xc+0.301, sphere.yc+0.001, toolpose.linear.z, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 0)
			# place ball
			plan = createPoint(plan, sphere.xc+0.30, sphere.yc, sphere.zc+0.001, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 0)
			plan = createPoint(plan, sphere.xc+0.301, sphere.yc+0.001, sphere.zc, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 1)
			plan = createPoint(plan, sphere.xc+0.30, sphere.yc, sphere.zc+0.001, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 0)
			# above ball
			plan = createPoint(plan, sphere.xc+0.300, sphere.yc, toolpose.linear.z, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 0)
			# move back to initial position
			plan = createPoint(plan, toolpose.linear.x, toolpose.linear.y, toolpose.linear.z, toolpose.angular.x, toolpose.angular.y, toolpose.angular.z, 0)
			print(plan)
		else:
			print("Beginning Path")
			while not rospy.is_shutdown() and toggleMove:
				# publish the plan
				plan_pub.publish(plan)
				# Wait for 0.1 seconds until the next loop and repeat
				loop_rate.sleep()
			
		loop_rate.sleep()
main()

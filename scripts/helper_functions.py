#!/usr/bin/env python

import rospy
from math import *
from geometry_msgs.msg import Pose, PoseArray, PointStamped, Quaternion, Point, Twist
from sensor_msgs.msg import LaserScan
import tf

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

def move_function (angle, dist):
	""" 
	Move function will move the robot in stage by publishing cmd_vel. 
	
	The robot first turns according to the given angle 
	& then moves forward for the given distance.

	Function Variables:
		angle => Angle the robot has to turn before it starts to move forward
		dist => Distance the robot needs to move forward after the angle is applied

	"""

	""" First Turn (in 10 degrees increments) """
	angle_in_rad = float(angle*pi/180.0)
	if (angle > 0):
		increment = 10	
	else:
		increment = -10

	increment_in_rad = float(increment*pi/180.0)
	while angle_in_rad != 0:
		twist = Twist()
		""" Stage Doesn't Handle Large Turns accurately"""
		if abs(angle_in_rad) > abs(increment_in_rad) :
			twist.angular.z = increment_in_rad
			angle_in_rad -= increment_in_rad
		else:
			twist.angular.z = angle_in_rad
			angle_in_rad = 0

		cmd_vel_pub.publish(twist)
		rospy.sleep(1)
		twist = Twist()
		cmd_vel_pub.publish(twist)


	""" Then Move """
	twist = Twist()
	twist.linear.x = dist
	cmd_vel_pub.publish(twist)
	rospy.sleep(1)
	twist = Twist()
	cmd_vel_pub.publish(twist)

def get_pose(x, y, theta):

	""" 
	Pose update function will return the pose of a particle given x,y,theta. 	
	- x, y will be directly used.
	- But as all odometry is 6DOF (Degrees of Freedom) we'll need a quaternion created from the euler angle
	"""

	pose = Pose()
	pose.position.x = x
	pose.position.y = y
	quat = tf.transformations.quaternion_from_euler(0, 0, theta)
	pose.orientation.x = quat[0]
	pose.orientation.y = quat[1]
	pose.orientation.z = quat[2]
	pose.orientation.w = quat[3]
	return pose

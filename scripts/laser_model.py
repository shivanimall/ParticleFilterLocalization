#!/usr/bin/env python
import rospy
import numpy as np
import random as r
from math import *
from read_config import read_config

from sensor_msgs.msg import LaserScan

class laserModel():
    def __init__(self):
    	rospy.init_node("laser_model")
    	self.config = read_config()
    	r.seed(self.config['seed'])
    	self.laser_sigma_hit = self.config['laser_sigma_hit']
    	rospy.Subscriber('base_scan', LaserScan, self.scan_callback)
    	self.base_scan_pub = rospy.Publisher('base_scan_with_error', LaserScan, queue_size = 10)
    	rospy.spin()

    def scan_callback(self, laser_scan):
		self.base_scan_msg = LaserScan()
		self.base_scan_msg.header = laser_scan.header
		self.base_scan_msg.angle_min = laser_scan.angle_min
		self.base_scan_msg.angle_max = laser_scan.angle_max
		self.base_scan_msg.angle_increment = laser_scan.angle_increment
		self.base_scan_msg.time_increment = laser_scan.time_increment
		self.base_scan_msg.scan_time = laser_scan.scan_time
		self.base_scan_msg.range_min = laser_scan.range_min
		self.base_scan_msg.range_max = laser_scan.range_max
		self.base_scan_msg.intensities = laser_scan.intensities
		for i in range(len(laser_scan.ranges)):
			if laser_scan.ranges[i] == laser_scan.range_max:
				self.base_scan_msg.ranges.append(laser_scan.ranges[i])
			else:
				self.base_scan_msg.ranges.append(laser_scan.ranges[i] + ceil(r.gauss(0, self.laser_sigma_hit)*100.)/100.)
		self.base_scan_pub.publish(self.base_scan_msg)


if __name__ == '__main__':
    lm = laserModel()

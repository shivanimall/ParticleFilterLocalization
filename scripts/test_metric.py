#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool
import json
import tf
from math import *
from read_config import read_config

class RobotLogger():
    def __init__(self):
        rospy.init_node("robot_logger")
        self.base_pose_ground_truth_sub = rospy.Subscriber(
                "base_pose_ground_truth",
                Odometry,
                self.update_ground_truth
        )
        self.particlecloud_sub = rospy.Subscriber(
                "particlecloud",
                PoseArray,
                self.update_particlecloud
        )
        self.json_update = rospy.Subscriber(
                "result_update",
                Bool,
                self.result_update
        )
        self.simulation_complete_sub = rospy.Subscriber(
                "sim_complete",
                Bool,
                self.handle_shutdown
        )
        self.init_files()
        self.config = read_config()
        self.num_particles = self.config['num_particles']
        rospy.spin()

    def init_files(self):
        with open('time_results.json', 'w+') as infile:
            pass
        open('std_dev_results.json', 'w+').close()
        open('metric_results.json', 'w+').close()
        self.time_data = []
        self.std_dev_x_data = []
        self.std_dev_y_data = []
        self.std_dev_angle_data = []
        self.std_dev_data = []
        self.metric_data = []
        self.init_time = float(rospy.Time.now().to_sec()/60)

    def update_ground_truth(self, message):
        self.base_truth_x = message.pose.pose.position.x
        self.base_truth_y = message.pose.pose.position.y
        quaternion = (message.pose.pose.orientation.x,
    				message.pose.pose.orientation.y,
    				message.pose.pose.orientation.z,
    				message.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.base_truth_angle = euler[2]

    def update_particlecloud(self, message):
    	self.particlecloud_poses = message.poses 

    def result_update(self, message):
        self.update_metric()
        with open('time_results.json', 'w') as time:
            time.write('{\n"time_elapsed" : ')
            json.dump(self.time_data, time)
            time.write('\n} \n')
        with open('std_dev_results.json', 'w') as std_dev:
            std_dev.write('{\n"std_dev_x" : ')
            json.dump(self.std_dev_x_data, std_dev)
            std_dev.write('\n} \n')
            std_dev.write('{\n"std_dev_y" : ')
            json.dump(self.std_dev_y_data, std_dev)
            std_dev.write('\n} \n')
            std_dev.write('{\n"std_dev_angle" : ')
            json.dump(self.std_dev_angle_data, std_dev)
            std_dev.write('\n} \n')
        with open('metric_results.json', 'w') as metric_file:
            metric_file.write('{\n"metric" : ')
            json.dump(self.metric_data, metric_file)
            metric_file.write('\n} \n')



    def update_metric(self):
    	time_elapsed_value = float(rospy.Time.now().to_sec()/60) - self.init_time #minutes
    	self.time_data.extend([time_elapsed_value])

    	var_x_value = 0.0
    	var_y_value = 0.0
    	var_angle_value = 0.0

    	#STD DEV
    	for particle_index in range(len(self.particlecloud_poses)):
    		pose = self.particlecloud_poses[particle_index]
    		var_x_value += (pose.position.x - self.base_truth_x)**2
    		var_y_value += (pose.position.y - self.base_truth_y)**2
    		quaternion = (pose.orientation.x,
						pose.orientation.y,
						pose.orientation.z,
						pose.orientation.w)
	        particle_euler = tf.transformations.euler_from_quaternion(quaternion)
    		var_angle_value += (particle_euler[2] - self.base_truth_angle)**2

    	norm_var_x = var_x_value/len(self.particlecloud_poses)
    	norm_var_y = var_y_value/len(self.particlecloud_poses)
    	norm_var_angle = var_angle_value/len(self.particlecloud_poses)

    	std_dev_x_value = sqrt(norm_var_x)
    	std_dev_y_value = sqrt(norm_var_y)
    	std_dev_angle_value = sqrt(norm_var_angle)

    	self.std_dev_x_data.extend([std_dev_x_value])
    	self.std_dev_y_data.extend([std_dev_y_value])
    	self.std_dev_angle_data.extend([std_dev_angle_value])

        #RMS
    	metric = (sqrt(self.std_dev_x_data[0]**2 + self.std_dev_y_data[0]**2)/sqrt(std_dev_x_value**2 + std_dev_y_value**2))+(self.std_dev_angle_data[0]/std_dev_angle_value)

        # if (time_elapsed_value > 10):
        #     metric = metric/(time_elapsed_value-9)


        #print std_dev_x_value, std_dev_y_value, std_dev_angle_value, metric

    	self.metric_data.extend([metric])

    def handle_shutdown(self, message):
        print "sim complete!", message.data

        self.update_metric()

        #particles_meeting_max_limit
        count = 0.0
        for particle_index in range(len(self.particlecloud_poses)):
            pose = self.particlecloud_poses[particle_index]
            diff_x_value = (pose.position.x - self.base_truth_x)
            diff_y_value = (pose.position.y - self.base_truth_y)
            diff_dist_val = sqrt(diff_x_value**2 + diff_y_value**2)
            if (diff_dist_val <= 55):
                count += 1
        #percentage
        self.particles_meeting_max_limit = float(count*100.0/self.num_particles)

        if message.data:
            with open('time_results.json', 'w') as time:
                time.write('{\n"time_elapsed" : ')
                json.dump(self.time_data, time)
                time.write('\n} \n')
            with open('std_dev_results.json', 'w') as std_dev:
                std_dev.write('{\n"std_dev_x" : ')
                json.dump(self.std_dev_x_data, std_dev)
                std_dev.write('\n} \n')
                std_dev.write('{\n"std_dev_y" : ')
                json.dump(self.std_dev_y_data, std_dev)
                std_dev.write('\n} \n')
                std_dev.write('{\n"std_dev_angle" : ')
                json.dump(self.std_dev_angle_data, std_dev)
                std_dev.write('\n} \n')
            with open('metric_results.json', 'w') as metric_file:
                metric_file.write('{\n"metric" : ')
                json.dump(self.metric_data, metric_file)
                metric_file.write('\n} \n')
            with open('max_limit_results.json', 'w') as max_limit_file:
                max_limit_file.write('{\n"max_limit_percentage" : ')
                json.dump(self.particles_meeting_max_limit, max_limit_file)
                max_limit_file.write('\n} \n')
            with open('basetruth.json', 'w') as bt_file:
                base_truth = []
                base_truth.append(self.base_truth_x)
                base_truth.append(self.base_truth_y)
                bt_file.write('{\n"basetruth" : ')
                json.dump(base_truth, bt_file)
                bt_file.write('\n} \n')


if __name__ == '__main__':
    rl = RobotLogger()

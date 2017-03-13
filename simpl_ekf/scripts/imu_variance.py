#!/usr/bin/env python

"""
    Name:           imu_variance.py
    Description:    Computes the variance of IMU data from ROS topic.
    Author:         Thomas Lew
    Last update:    2017.03.12
                                                    					"""
import rospy
import sys

from sensor_msgs.msg import Imu
import numpy as np

IMU_TOT   = 100
imu_count = 0

#	IMU data is stored
data_gyro_x  = []
data_gyro_y  = []
data_gyro_z  = []
data_accel_x = []
data_accel_y = []
data_accel_z = []

def callback(data):
	global imu_count
	imu_count += 1
	rospy.loginfo("New Imu values received! Num: " + str(imu_count))

	data_gyro_x.append(data.angular_velocity.x)
	data_gyro_y.append(data.angular_velocity.y)
	data_gyro_z.append(data.angular_velocity.z)

	data_accel_x.append(data.linear_acceleration.x)
	data_accel_y.append(data.linear_acceleration.y)
	data_accel_z.append(data.linear_acceleration.z)
	
	if imu_count == IMU_TOT:
		rospy.loginfo("NEW CALCULATION OF MEAN AND VARIANCE")

		mean_gyro_x  = np.mean(data_gyro_x)
		mean_gyro_y  = np.mean(data_gyro_y)
		mean_gyro_z  = np.mean(data_gyro_z)
		mean_accel_x = np.mean(data_accel_x)
		mean_accel_y = np.mean(data_accel_y)
		mean_accel_z = np.mean(data_accel_z)

		rospy.loginfo("Mean of gyro_x   " + str(mean_gyro_x))
		rospy.loginfo("Mean of gyro_y   " + str(mean_gyro_y))
		rospy.loginfo("Mean of gyro_z   " + str(mean_gyro_z))
		rospy.loginfo("Mean of accel_x  " + str(mean_accel_x))
		rospy.loginfo("Mean of accel_y  " + str(mean_accel_y))
		rospy.loginfo("Mean of accel_z  " + str(mean_accel_z))

		var_gyro_x  = np.var(data_gyro_x)
		var_gyro_y  = np.var(data_gyro_y)
		var_gyro_z  = np.var(data_gyro_z)
		var_accel_x = np.var(data_accel_x)
		var_accel_y = np.var(data_accel_y)
		var_accel_z = np.var(data_accel_z)

		rospy.loginfo("Variance of gyro_x   " + str(var_gyro_x))
		rospy.loginfo("Variance of gyro_y   " + str(var_gyro_y))
		rospy.loginfo("Variance of gyro_z   " + str(var_gyro_z))
		rospy.loginfo("Variance of accel_x  " + str(var_accel_x))
		rospy.loginfo("Variance of accel_y  " + str(var_accel_y))
		rospy.loginfo("Variance of accel_z  " + str(var_accel_z))

		imu_count = 0


def variance_calculator():
	#	To enable multiple ROS nodes listeners, "anonymous = True" is necessary
	rospy.init_node('Python_IMU_variance_calculator', anonymous = True)

	#	Choose the topic to subscribe to
	rospy.Subscriber("/imu_3dm_gx4/imu", Imu, callback)
	#rospy.Subscriber("/python_IMU", Imu, callback)

	rospy.spin()

if __name__ == '__main__':
	variance_calculator()

""" Sources
	http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)#rospy_tutorials.2BAC8-Tutorials.2BAC8-
		WritingPublisherSubscriber.Writing_the_Publisher_Node 
"""

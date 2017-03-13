#!/usr/bin/env python

"""
    Name:           imu_publisher.py
    Description:    ROS IMU Publisher for Kalman Filters testing.
    Author:         Thomas Lew
    Last update:    2017.03.12
                                                    """
import rospy
import sys
import numpy as numpy	#for noise

from sensor_msgs.msg import Imu
from time import time, sleep

def talker():
    QUEUE_SIZE          = 10
    RATE_PUBLISHER      = 50     # Frequency in Hz

    SIZE_VECTOR         = 3
    MEAN_ACCEL          = 0
    MEAN_GYRO           = 0
    STD_DEVIATION_ACCEL = 0.2
    STD_DEVIATION_GYRO  = 0.1

    INITIAL_YAW         = 0
    INITIAL_PITCH       = 0
    INITIAL_ROLL        = 0

    G                   = 9.81

    #   ROS Setup
    pub = rospy.Publisher('python_IMU', Imu, queue_size = QUEUE_SIZE)
    rospy.init_node('Python_IMU_Publisher_node')
    rate = rospy.Rate(RATE_PUBLISHER)

    while not rospy.is_shutdown():
        print('Publishing new IMU Data')

        yaw   = INITIAL_YAW
        pitch = INITIAL_PITCH
        roll  = INITIAL_ROLL

        imuMsg = Imu()
        imuMsg.orientation_covariance = [0 , 0 , 0, 0, 0, 0, 0, 0, 0]
        imuMsg.angular_velocity_covariance = [0, 0 , 0, 0 , 0, 0, 0 , 0 , 0]
        imuMsg.linear_acceleration_covariance = [0 , 0 , 0, 0 , 0, 0, 0 , 0 , 0]

        #   Noise is modelled as a gaussian distribution
        noise_accel = numpy.random.normal(MEAN_ACCEL, STD_DEVIATION_ACCEL,   SIZE_VECTOR)
        noise_gyro  = numpy.random.normal(MEAN_GYRO, STD_DEVIATION_GYRO,    SIZE_VECTOR)


        # ------------------------------------------ #
        # Publish IMU data with added gaussian noise
        # ------------------------------------------ #

        #   Accelerometer measurements
        imuMsg.linear_acceleration.x = 0 + noise_accel[0]
        imuMsg.linear_acceleration.y = 0 + noise_accel[1]
        imuMsg.linear_acceleration.z = G + noise_accel[2]

        #   Gyroscope measurements
        imuMsg.angular_velocity.x = 0 + noise_gyro[0]
        imuMsg.angular_velocity.y = 0 + noise_gyro[1]
        imuMsg.angular_velocity.z = 0 + noise_gyro[2]

        #   Quaternion
        imuMsg.orientation.x = 0
        imuMsg.orientation.y = 0
        imuMsg.orientation.z = 0
        imuMsg.orientation.w = 1

        #   Time and frame name
        imuMsg.header.stamp = rospy.Time.now()
        imuMsg.header.frame_id = 'base_link'

        #   Publish and wait to achieve the correct frequency
        pub.publish(imuMsg)
        rate.sleep()

try:
    print('try')
    talker()
except rospy.ROSInterruptException:
    pass



# Sources:
# http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(python)#rospy_tutorials.2BAC8-Tutorials.2BAC8-WritingPublisherSubscriber.Writing_the_Publisher_Node
# http://www.programcreek.com/python/example/14775/rospy.Publisher 
# http://answers.ros.org/question/66026/python-publisher-and-c-subscriber-problem/
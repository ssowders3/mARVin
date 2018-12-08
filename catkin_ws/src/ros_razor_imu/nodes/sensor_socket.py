#!/usr/bin/env python

import socket, traceback
import rospy
import string
import math
import sys
import pdb
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server

# Phone Orientation Guidelines
# 1. Place phone flat against surface with screen face up for correct Z orientation
# 2. top short side of phone = left side of y axis i.e. Left side up = +9.8 for Accel
# 3. left long side of phone = front side of x axis i.e. front side down = -9.8 for Accel


# Basic Socket Setup
host = '143.215.104.26'
port = 5555

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
s.bind((host, port))

# ROS Node Setup
rospy.init_node("imu_node")
pub = rospy.Publisher('imu_data', Imu, queue_size=1)

imuMsg = Imu()
seq = 0

# Covariance Matrices setup
imuMsg.orientation_covariance = [
0.0025 , 0 , 0,
0, 0.0025, 0,
0, 0, 0.0025
]

imuMsg.angular_velocity_covariance = [
0.02, 0 , 0,
0 , 0.02, 0,
0 , 0 , 0.02
]

imuMsg.linear_acceleration_covariance = [
0.04 , 0 , 0,
0 , 0.04, 0,
0 , 0 , 0.04
]


while not rospy.is_shutdown():
    try:
        message, address = s.recvfrom(8192)
        temp = message.split(',')
        if len(temp) == 13 and ' 81' in temp:
            # setting Linear Accel
            # Value for X axis set to negative to fit with robot_localization expectation
            imuMsg.linear_acceleration.x = -float(temp[2])
            imuMsg.linear_acceleration.y = float(temp[3])
            imuMsg.linear_acceleration.z = float(temp[4])
 
            # setting Angular Velocity
            imuMsg.angular_velocity.x = float(temp[6])
            imuMsg.angular_velocity.y = float(temp[7])
            imuMsg.angular_velocity.z = float(temp[8])

            # setting Orientation
            yaw = float(temp[10])
            pitch = float(temp[11])
            roll = float(temp[12])

            q = quaternion_from_euler(roll,pitch,yaw)
            imuMsg.orientation.x = q[0]
            imuMsg.orientation.y = q[1]
            imuMsg.orientation.z = q[2]
            imuMsg.orientation.w = q[3]
         
            # Setting imuMsg data
            imuMsg.header.stamp = rospy.Time.now()
            imuMsg.header.frame_id = 'base_footprint'
            imuMsg.header.seq = seq
            seq = seq+1
            pub.publish(imuMsg)


    except (KeyboardInterrupt, SystemExit):
        raise
    except:
        traceback.print_exc()

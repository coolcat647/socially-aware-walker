#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import time
import copy
import numpy as np

import rospy
from geometry_msgs.msg import WrenchStamped, Vector3, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry
import message_filters

# Global variables
flag_writing_busy = False

last_robot_x = -1.0
last_robot_y = -1.0
last_robot_theta = -1.0
last_time = None
last_v = 0
last_omega = 0

exported_file = None

def callback(force_msg, odom_msg):
    global last_robot_x
    global last_robot_y
    global last_robot_theta
    global last_time
    global last_v
    global last_omega
    global exported_file
    
    if last_time is None:
        last_time = rospy.Time.now()
        last_robot_x = odom_msg.pose.pose.position.x
        last_robot_y = odom_msg.pose.pose.position.y
        euler_angle = euler_from_quaternion([odom_msg.pose.pose.orientation.x, 
                                            odom_msg.pose.pose.orientation.y, 
                                            odom_msg.pose.pose.orientation.z, 
                                            odom_msg.pose.pose.orientation.w])
        last_robot_theta = euler_angle[2]
        last_v = 0
        last_omega = 0

        # Prepare output file
        exported_file = open('force_data.csv', 'a', encoding='utf-8')
        return

    now_time = rospy.Time.now()
    dt = (now_time - last_time).to_sec()
    if dt > 1.0:
        last_time = rospy.Time.now()
        last_robot_x = odom_msg.pose.pose.position.x
        last_robot_y = odom_msg.pose.pose.position.y
        euler_angle = euler_from_quaternion([odom_msg.pose.pose.orientation.x, 
                                            odom_msg.pose.pose.orientation.y, 
                                            odom_msg.pose.pose.orientation.z, 
                                            odom_msg.pose.pose.orientation.w])
        last_robot_theta = euler_angle[2]
        last_v = 0
        last_omega = 0

    # Odom msg process
    x_dot = (odom_msg.pose.pose.position.x - last_robot_x) / dt
    y_dot = (odom_msg.pose.pose.position.y - last_robot_y) / dt
    v = np.sqrt(x_dot * x_dot + y_dot * y_dot)
    euler_angle = euler_from_quaternion([odom_msg.pose.pose.orientation.x, 
                                            odom_msg.pose.pose.orientation.y, 
                                            odom_msg.pose.pose.orientation.z, 
                                            odom_msg.pose.pose.orientation.w])
    omega = (euler_angle[2] - last_robot_theta) / dt

    # Force msg process
    force_y = force_msg.wrench.force.y
    torque_z = force_msg.wrench.torque.z

    output_text = "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\n".format(last_v, last_omega, force_y, torque_z, v, omega)
    exported_file.write(output_text)

    # Update "last variable"
    last_time = now_time
    last_robot_x = odom_msg.pose.pose.position.x
    last_robot_y = odom_msg.pose.pose.position.y
    last_robot_theta = euler_angle[2]
    last_v = v
    last_omega = omega


def shutdown_cb():
    global exported_file
    exported_file.close()
    rospy.loginfo("Shutdown " + rospy.get_name())


if __name__ == '__main__':
    rospy.init_node('force_v_extraction', anonymous=False)
    rospy.on_shutdown(shutdown_cb)
    sub_force = message_filters.Subscriber("force_filtered", WrenchStamped)
    sub_odom = message_filters.Subscriber("wheel_odom", Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([sub_force, sub_odom], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.loginfo(rospy.get_name() + ' is ready.')
    rospy.spin()

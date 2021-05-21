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

last_time = None
last_v = 0
last_w = 0
v_dot = 0
w_dot = 0

exported_file = None

def callback(force_msg, odom_msg):
    global last_time
    global last_v
    global last_w
    global exported_file
    
    if last_time is None:
        last_time = rospy.Time.now()
        last_v = 0
        last_w = 0
        v_dot = 0
        w_dot = 0

        # Prepare output file
        exported_file = open('force_data.csv', 'a', encoding='utf-8')
        output_text = "dt, v, w, force_y, torque_z, v_dot, w_dot\n"
        exported_file.write(output_text)
        return

    now_time = rospy.Time.now()
    dt = (now_time - last_time).to_sec()
    if dt > 1.0:
        last_time = rospy.Time.now()
        last_v = 0
        last_w = 0
        v_dot = 0
        w_dot = 0

    # Odom msg process
    v = odom_msg.twist.twist.linear.x
    w = odom_msg.twist.twist.angular.z
    v_dot = v - last_v
    w_dot = w - last_w

    # Force msg process
    force_y = force_msg.wrench.force.y
    torque_z = force_msg.wrench.torque.z

    output_text = "{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}\n".format(dt, v, w, force_y, torque_z, v_dot, w_dot)
    exported_file.write(output_text)

    # Update "last variable"
    last_time = now_time
    last_v = v
    last_w = w


def shutdown_cb():
    global exported_file
    exported_file.close()
    rospy.loginfo("Shutdown " + rospy.get_name())


if __name__ == '__main__':
    rospy.init_node('force_v_extraction', anonymous=False)
    rospy.on_shutdown(shutdown_cb)
    sub_force = message_filters.Subscriber("/walker/force_filtered", WrenchStamped)
    sub_odom = message_filters.Subscriber("/walker/wheel_odom", Odometry)
    ts = message_filters.ApproximateTimeSynchronizer([sub_force, sub_odom], 10, 0.1, allow_headerless=True)
    ts.registerCallback(callback)

    rospy.loginfo(rospy.get_name() + ' is ready.')
    rospy.spin()

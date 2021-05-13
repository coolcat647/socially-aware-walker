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
from nav_msgs.msg import Odometry
import message_filters

MIN_ENABLE_FORCE = 6.0
MIN_ENABLE_TORQUE = 0.8

MAX_LINEAR_VELOCITY = 0.6
MAX_ANGULAR_VELOCITY = 1.0

MASS = 30.0
MOMENT_OF_INERTIA = 10.0
DAMPING_XY = 45.0 
DAMPING_THETA = 20
CONSTANT_FRACTION_XY = 1e-3
CONSTANT_FRACTION_THETA = 1e-3

class Force2CmdNode(object):
    def __init__(self):
        self.last_time = None
        self.last_v = 0
        self.last_w = 0

        self.pub_vel = rospy.Publisher('/walker/cmd_vel', Twist, queue_size=1)
        sub_force = message_filters.Subscriber("/walker/force_filtered", WrenchStamped)
        sub_odom = message_filters.Subscriber("/walker/wheel_odom", Odometry)
        ts = message_filters.ApproximateTimeSynchronizer([sub_force, sub_odom], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

        rospy.loginfo(rospy.get_name() + ' is ready.')
        

    def callback(self, force_msg, odom_msg):
        if self.last_time is None:
            self.last_time = rospy.Time().now()
            self.last_v = 0
            self.last_w = 0
            return

        force_y = (force_msg.wrench.force.y ) if np.abs(force_msg.wrench.force.y) > MIN_ENABLE_FORCE else 0.0
        torque_z = force_msg.wrench.torque.z if np.abs(force_msg.wrench.torque.z) > MIN_ENABLE_TORQUE else 0.0

        total_mass = MASS + (-force_msg.wrench.force.z / 9.8) 

        now_time = rospy.Time().now()
        dt = (now_time - self.last_time).to_sec()
        
        # Calculate target velocity by human force
        cmd_msg = Twist()
        cmd_msg.angular.z = np.clip(self.last_w, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
        cmd_msg.linear.x = np.clip(self.last_v, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
        self.pub_vel.publish(cmd_msg)

        v_dot = force_y / total_mass - self.last_v * DAMPING_XY / total_mass - CONSTANT_FRACTION_XY
        w_dot = torque_z / MOMENT_OF_INERTIA - self.last_w * DAMPING_THETA / MOMENT_OF_INERTIA - CONSTANT_FRACTION_THETA
        next_w = np.clip(self.last_w + w_dot * dt, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
        next_v = np.clip(self.last_v + v_dot * dt, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
        
        rospy.loginfo("v': {:.3f}, w': {:.3f}".format(self.last_v , self.last_w))
        rospy.loginfo("v : {:.3f}, w : {:.3f}".format(odom_msg.twist.twist.linear.x , odom_msg.twist.twist.angular.z))
        rospy.loginfo("")
        
        self.last_time = now_time
        self.last_v = next_v
        self.last_w = next_w


if __name__ == '__main__':
    rospy.init_node('force2cmd_node', anonymous=False)
    node = Force2CmdNode()
    rospy.spin()
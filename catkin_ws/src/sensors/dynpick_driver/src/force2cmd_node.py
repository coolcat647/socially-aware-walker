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

MIN_ENABLE_FORCE = 5.0
MIN_ENABLE_TORQUE = 2.0

MAX_LINEAR_VELOCITY = 0.6
MAX_ANGULAR_VELOCITY = 1.4

MASS = 22.771158961  * 0.1  #10.0 # 30.0
MOMENT_OF_INERTIA = 22.468527211 * 0.1 # 1.3 # 2.61
DAMPING_XY = 245.568732467  * 0.1 # 40.0 # 40.0 
DAMPING_THETA = 245.093435376 * 0.1 # 8.0

DESIRE_V = 0.2
DESIRE_OMEGA = 0.0

class Force2CmdNode(object):
    def __init__(self):
        self.last_time = None
        self.last_linear_velocity = 0
        self.last_angular_velocity = 0

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # self.sub_force = rospy.Subscriber("/force_filtered", WrenchStamped, self.force_cb, queue_size=1)
        sub_force = message_filters.Subscriber("force_filtered", WrenchStamped)
        sub_odom = message_filters.Subscriber("wheel_odom", Odometry)
        ts = message_filters.ApproximateTimeSynchronizer([sub_force, sub_odom], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)

        rospy.loginfo(rospy.get_name() + ' is ready.')
        

    def callback(self, force_msg, odom_msg):
        if self.last_time is None:
            self.last_time = rospy.Time().now()
            return

        force_y = force_msg.wrench.force.y if np.abs(force_msg.wrench.force.y) > MIN_ENABLE_FORCE else 0.0
        torque_z = force_msg.wrench.torque.z if np.abs(force_msg.wrench.torque.z) > MIN_ENABLE_TORQUE else 0.0

        now_time = rospy.Time().now()
        dt = (now_time - self.last_time).to_sec()
        
        # Calculate target velocity by human force
        # linear_velocity = (force_y / MASS * dt + self.last_linear_velocity) / (1 + DAMPING_XY / MASS * dt)
        # angular_velocity = (torque_z / MOMENT_OF_INERTIA * dt + self.last_angular_velocity) / (1 + DAMPING_THETA / MOMENT_OF_INERTIA * dt)
        
        linear_velocity = (force_y / 15.145 * dt + self.last_linear_velocity) / (15.145 + (-151.03) * dt)
        angular_velocity = (torque_z / 1.1801 * dt + self.last_angular_velocity) / (15.145 + (-7.975) * dt)
        
        # linear_velocity = self.last_linear_velocity * 0.9087 * 0.7 + force_y * 0.0028 * 1.5
        # angular_velocity = self.last_angular_velocity * 0.9028 * 0.5 + torque_z * 0.0051 * 7.5
        
        cmd_msg = Twist()
        cmd_msg.linear.x = np.clip(linear_velocity, -MAX_LINEAR_VELOCITY, MAX_LINEAR_VELOCITY)
        cmd_msg.angular.z = np.clip(angular_velocity, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
        rospy.loginfo("v, omega = ({:.2f},{:.2f})".format(cmd_msg.linear.x, cmd_msg.angular.z))
        self.pub_vel.publish(cmd_msg)
        
        self.last_time = now_time
        # self.last_linear_velocity = linear_velocity
        # self.last_angular_velocity = angular_velocity

        self.last_linear_velocity = odom_msg.twist.twist.linear.x
        self.last_angular_velocity = odom_msg.twist.twist.angular.z


if __name__ == '__main__':
    rospy.init_node('force2cmd_node', anonymous=False)
    node = Force2CmdNode()
    rospy.spin()
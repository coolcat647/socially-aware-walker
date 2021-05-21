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


class Force2CmdNode(object):
    def __init__(self):
        self.last_time = None
        self.last_v = 0
        self.last_w = 0

        # Load robot dynamics and constraints from parameter server
        self.robot_dynamics_dict = rospy.get_param('dynamics')
        self.robot_constraints_dict = rospy.get_param('constraints')
        
        self.pub_vel = rospy.Publisher('user_contributed/cmd_vel', Twist, queue_size=1)
        self.sub_force = rospy.Subscriber("force_filtered", WrenchStamped, self.force_cb, queue_size=1)
        rospy.loginfo(rospy.get_name() + ' is ready.')


    def force_cb(self, force_msg):
        if self.last_time is None:
            self.last_time = rospy.Time().now()
            self.last_v = 0
            self.last_w = 0
            return

        cmd_msg = Twist()
        cmd_msg.angular.z = self.last_w
        cmd_msg.linear.x = self.last_v
        self.pub_vel.publish(cmd_msg)

        # Calculate target velocity by human force
        now_time = rospy.Time().now()
        dt = (now_time - self.last_time).to_sec()
        
        force_y = force_msg.wrench.force.y if np.abs(force_msg.wrench.force.y) > self.robot_constraints_dict["min_enable_force"] else 0.0
        torque_z = force_msg.wrench.torque.z if np.abs(force_msg.wrench.torque.z) > self.robot_constraints_dict["min_enable_torque"] else 0.0
        total_mass = self.robot_dynamics_dict["mass"] + (-force_msg.wrench.force.z / 9.8)       # Newton -> kg
        
        v_dot = force_y / total_mass \
                - self.last_v * self.robot_dynamics_dict["damping_xy"] / total_mass \
                - self.robot_dynamics_dict["constant_fraction_xy"]

        w_dot = torque_z / self.robot_dynamics_dict["moment_of_inertia"] \
                - self.last_w * self.robot_dynamics_dict["damping_theta"] / self.robot_dynamics_dict["moment_of_inertia"] \
                - self.robot_dynamics_dict["constant_fraction_theta"]

        next_w = np.clip(self.last_w + w_dot * dt, -self.robot_constraints_dict["max_angular_velocity"], self.robot_constraints_dict["max_angular_velocity"])
        # next_v = np.clip(self.last_v + v_dot * dt, -self.robot_constraints_dict["max_linear_velocity"], self.robot_constraints_dict["max_linear_velocity"])
        next_v = np.clip(self.last_v + v_dot * dt, 0.0, self.robot_constraints_dict["max_linear_velocity"])
        # rospy.loginfo("v': {:.3f}, w': {:.3f}".format(self.last_v , self.last_w))
        
        # Update last v, w
        self.last_time = now_time
        self.last_v = next_v
        self.last_w = next_w


if __name__ == '__main__':
    rospy.init_node('force2cmd_node', anonymous=False)
    node = Force2CmdNode()
    rospy.spin()
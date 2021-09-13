#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
from std_srvs.srv import Empty, EmptyRequest

def goal_cb(msg):
    pedsim_unpause(EmptyRequest())


if __name__ == '__main__':
    rospy.init_node("gym_robot_synchronizer", anonymous=False)

    # Service Client
    rospy.wait_for_service("/pedsim_simulator/unpause_simulation", timeout=30)
    pedsim_unpause = rospy.ServiceProxy("/pedsim_simulator/unpause_simulation", Empty)

    # ROS subscriber
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_cb, queue_size=1)
    rospy.spin()
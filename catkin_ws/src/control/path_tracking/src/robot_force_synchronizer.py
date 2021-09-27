#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import Pose2D, PoseStamped
from std_srvs.srv import Empty, EmptyRequest
from walker_msgs.srv import PushingActionTrigger, PushingActionTriggerRequest

def goal_cb(msg):
    req = PushingActionTriggerRequest()
    req.duration_in_seconds = 120.0
    trig_pushing_action(req)


if __name__ == '__main__':
    rospy.init_node("robot_force_synchronizer", anonymous=False)

    # Service Client
    rospy.wait_for_service("/walker/trig_pushing_action", timeout=30)
    trig_pushing_action = rospy.ServiceProxy("/walker/trig_pushing_action", PushingActionTrigger)

    # ROS subscriber
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_cb, queue_size=1)
    rospy.spin()
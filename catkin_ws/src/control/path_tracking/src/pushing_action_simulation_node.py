#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import time
import copy
import numpy as np

import rospy
from std_srvs.srv import Trigger, TriggerResponse
from walker_msgs.srv import PushingActionTrigger, PushingActionTriggerResponse
from geometry_msgs.msg import WrenchStamped, Vector3, Twist
from nav_msgs.msg import Odometry

# The period of single-leg contacting the ground
SINGLE_GAIT_PERIOD = 5.0
MAX_PUSHING_FORCE_VALUE = 16.0 + 14.0
MIN_PUSHING_FORCE_VALUE = 6.0
UP_SLOPE_TIME_RATIO = 2 / 7
UP_SLOPE = (MAX_PUSHING_FORCE_VALUE - MIN_PUSHING_FORCE_VALUE) / (SINGLE_GAIT_PERIOD / 2 * UP_SLOPE_TIME_RATIO)
DOWN_SLOPE = (MIN_PUSHING_FORCE_VALUE - MAX_PUSHING_FORCE_VALUE) / (SINGLE_GAIT_PERIOD / 2 * (1.0 - UP_SLOPE_TIME_RATIO))

current_slope = 0.0
simulated_force_frequency = 0.0
force_y_value = 0.0
force_start_time = None
force_duration = None
pub_force = None

# Force wave
#
#          /\    /\    /\    /\    /\
#         /  \  /  \  /  \  /  \  /  \
#        /    \/    \/    \/    \/    \
#       /    
#      /       <---------->
# ____/      SINGLE_GAIT_PERIOD

def trigger_cb(req):
    global force_duration
    global force_start_time
    global current_slope

    resp = PushingActionTriggerResponse()
    if req.duration_in_seconds <= 0:
        rospy.logerr("Invalid force duration is given: {}, skip this trigger event.".format(req.duration_in_seconds))
        resp.success = False
    else:        
        force_duration = req.duration_in_seconds
        force_start_time = rospy.Time().now()
        current_slope = UP_SLOPE * 2
        resp.success = True
    return resp


def timer_cb(event):
    global current_slope
    global force_y_value

    force_msg = WrenchStamped()

    if force_start_time is not None and (rospy.Time.now() - force_start_time).to_sec() <= force_duration:
        if current_slope > 0 and force_y_value >= MAX_PUSHING_FORCE_VALUE:
            current_slope = DOWN_SLOPE
        elif current_slope < 0 and force_y_value <= MIN_PUSHING_FORCE_VALUE:
            current_slope = UP_SLOPE
        force_y_value += (current_slope / simulated_force_frequency + (np.random.rand(1) - 0.5)) 
    else:
        force_y_value = (np.random.rand(1) - 0.5) 

    force_msg.wrench.force.y = force_y_value
    force_msg.header.frame_id = "force_sensor_link"
    force_msg.header.stamp = rospy.Time().now()
    pub_force.publish(force_msg)


if __name__ == '__main__':
    rospy.init_node('pushing_action_simulation_node', anonymous=False)
    pub_force = rospy.Publisher("force_filtered", WrenchStamped, queue_size=1)
    srv_trigger = rospy.Service('trig_pushing_action', PushingActionTrigger, trigger_cb)

    simulated_force_frequency = 20
    rospy.Timer(rospy.Duration(1.0 / simulated_force_frequency), timer_cb)
    rospy.loginfo(rospy.get_name() + ' is ready.')
    rospy.spin()
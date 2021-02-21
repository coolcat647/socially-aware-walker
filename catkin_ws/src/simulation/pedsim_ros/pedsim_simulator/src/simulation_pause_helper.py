#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import numpy as np
import rospy

import dynamic_reconfigure.client


def callback(config):
    rospy.logwarn("\"paused\" is set to: {}".format(config['paused']))

if __name__ == '__main__':
    rospy.init_node('robot_position_setup_helper', anonymous=False)
    client = dynamic_reconfigure.client.Client("pedsim_simulator", timeout=30, config_callback=callback)
    client.update_configuration({"paused": True, })

#!/usr/bin/env python

import numpy as np

import rospy
from geometry_msgs.msg import Pose2D
from pedsim_msgs.msg  import GymAgentInfo
from pedsim_srvs.srv import GymReset, GymResetRequest, GymResetResponse 

if __name__ == '__main__':
    rospy.init_node("gym_reset_test")

    # Service Client
    rospy.wait_for_service("/pedsim_simulator/gym_reset", timeout=30)
    gym_reset = rospy.ServiceProxy("/pedsim_simulator/gym_reset", GymReset)

    num_agents = 5
    moving_distance = 5.0

    all_wp_list = []
    req = GymResetRequest()
    for i in range(num_agents):
        theta = np.random.rand() * np.pi * 2
        init_pose2d = Pose2D(x=np.cos(theta) * moving_distance,
                             y=np.sin(theta) * moving_distance,
                             theta=theta)
        wp1 = Pose2D(x=np.cos(theta - np.pi) * moving_distance,
                     y=np.sin(theta - np.pi) * moving_distance,
                     theta=-init_pose2d.theta)
        while True:
            flag_collision = False
            for other_wp in all_wp_list:
                if np.hypot(wp1.x - other_wp.x, wp1.y - other_wp.y) < 1.0:
                    flag_collision = True
                    break
            if not flag_collision: break
            else:
                theta = np.random.rand() * np.pi * 2
                init_pose2d = Pose2D(x=np.cos(theta) * moving_distance,
                                     y=np.sin(theta) * moving_distance,
                                     theta=theta)
                wp1 = Pose2D(x=np.cos(theta - np.pi) * moving_distance,
                             y=np.sin(theta - np.pi) * moving_distance,
                             theta=-init_pose2d.theta)
        all_wp_list.append(wp1)


        agent_info = GymAgentInfo(init_pose2d=init_pose2d, waypoints_list=[wp1])
        req.agents_list.append(agent_info)

    gym_reset(req)
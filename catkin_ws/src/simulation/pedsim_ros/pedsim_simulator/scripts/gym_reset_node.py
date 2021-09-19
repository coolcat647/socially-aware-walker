#!/usr/bin/env python3
import os
import argparse
import numpy as np
import csv

import rospy
import rospkg
from geometry_msgs.msg import Pose2D
from pedsim_msgs.msg  import GymAgentInfo
from pedsim_srvs.srv import GymReset, GymResetRequest, GymResetResponse
from gazebo_msgs.srv import GetWorldProperties

MOVING_DISTANCE = 4.0


if __name__ == '__main__':
    rospy.init_node("gym_reset_node")

    # Convert arguments
    parser = argparse.ArgumentParser(description='Reset the humans of Pedsim like OpenAI Gym environment.')
    parser.add_argument('--use_testcase', default=False, action='store_true')
    parser.add_argument('--num_agents', type=int, default='8')
    parser.add_argument('--testcase', type=int, default=1)
    tmp_args = rospy.myargv()   # args which included 'ros related' arguments, need to be filtered
    args = parser.parse_args(tmp_args[1:])

    # Gym reset service client: 
    rospy.wait_for_service("/pedsim_simulator/gym_reset", timeout=3)
    gym_reset = rospy.ServiceProxy("/pedsim_simulator/gym_reset", GymReset)
    # Gazebo world info service client: gym_reset
    rospy.wait_for_service("/gazebo/get_world_properties", timeout=3)
    get_gazebo_world_info = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
    world_info = get_gazebo_world_info()
    actors_list = [actor_name for actor_name in world_info.model_names if "actor" in actor_name]

    # Testcase humans number problem handling
    if args.use_testcase and len(actors_list) != args.num_agents:
        rospy.logerr("Testcase humans number and humans number on current gazebo are not matched.")
        exit(-1)
    elif args.use_testcase and (args.testcase <= 0 or args.testcase > 500):
        rospy.logerr("Testcase:{} is out of range.".format(args.testcase))
        exit(-1)

    if args.use_testcase:
        # Find csv file
        rospack = rospkg.RosPack()
        gym_config_filepath = os.path.join(rospack.get_path("pedsim_simulator"),
                                       "scenarios/gym_env_config",
                                       "testcases_{}_humans.csv".format(args.num_agents))
        # Check if the test case file exists or not
        if not os.path.isfile(gym_config_filepath):
            rospy.logerr("file: {} is not existed".format(gym_config_filepath))
            exit(-1)

        # Read testcase from csv file
        testcase_dict = {}
        with open(gym_config_filepath, newline='') as csvfile:
            rows = list(csv.reader(csvfile, delimiter=','))
            header_row = rows[0]
            target_row = rows[args.testcase]
            for idx in range(len(target_row)):
                if target_row[idx] != "":
                    testcase_dict[header_row[idx]] = float(target_row[idx])
            # print(testcase_dict)

        # Set ROS parameter: testcase
        rospy.set_param('/walker/testcase', args.testcase)

        req = GymResetRequest()
        for idx in range(args.num_agents):
            init_pose2d = Pose2D(x=testcase_dict['px{}'.format(idx + 1)],
                                 y=testcase_dict['py{}'.format(idx + 1)],
                                 theta=0)
            wp1 = Pose2D(x=testcase_dict['gx{}'.format(idx + 1)],
                         y=testcase_dict['gy{}'.format(idx + 1)],
                         theta=0)
            init_pose2d.theta = np.arctan2(wp1.y - init_pose2d.y, 
                                           wp1.x - init_pose2d.x)
            agent_info = GymAgentInfo(init_pose2d=init_pose2d, waypoints_list=[wp1])
            req.agents_list.append(agent_info)
        gym_reset(req)    

    else:
        # Set ROS parameter: testcase
        rospy.set_param('/walker/testcase', 0)

        # Randomly generate human's position and waypoint
        all_wp_list = []
        all_wp_list.append(Pose2D(x=MOVING_DISTANCE, y=0, theta=0))     # add robot destination first
        req = GymResetRequest()
        for i in range(args.num_agents):
            # Randomize the pedestrian's angle
            theta = np.random.rand() * np.pi * 2
            init_pose2d = Pose2D(x=np.cos(theta) * MOVING_DISTANCE,
                                 y=np.sin(theta) * MOVING_DISTANCE,
                                 theta=theta - np.pi)
            wp1 = Pose2D(x=np.cos(theta - np.pi) * MOVING_DISTANCE,
                         y=np.sin(theta - np.pi) * MOVING_DISTANCE,
                         theta=theta - np.pi)
            # If waypoints are overlapping, re-generate a new position
            while True:
                flag_collision = False
                for other_wp in all_wp_list:
                    if np.hypot(wp1.x - other_wp.x, wp1.y - other_wp.y) < 1.0:
                        flag_collision = True
                        break
                if not flag_collision: break
                else:
                    theta = np.random.rand() * np.pi * 2
                    init_pose2d = Pose2D(x=np.cos(theta) * MOVING_DISTANCE,
                                         y=np.sin(theta) * MOVING_DISTANCE,
                                         theta=theta - np.pi)
                    wp1 = Pose2D(x=np.cos(theta - np.pi) * MOVING_DISTANCE,
                                 y=np.sin(theta - np.pi) * MOVING_DISTANCE,
                                 theta=theta - np.pi)
            all_wp_list.append(wp1)

            agent_info = GymAgentInfo(init_pose2d=init_pose2d, waypoints_list=[wp1])
            req.agents_list.append(agent_info)
        gym_reset(req)

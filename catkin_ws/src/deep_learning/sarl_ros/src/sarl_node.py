#!/usr/bin/env python3

import logging
import argparse
import configparser
import os
import sys
import torch
import numpy as np
import gym
import matplotlib.pyplot as plt
import imp

import rospy
import rospkg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point, PointStamped, Pose2D
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path, Odometry
from walker_msgs.msg import Trk3DArray, Trk3D

from crowd_nav.utils.explorer import Explorer
from crowd_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA
from crowd_sim.envs.utils.state import ObservableState
from crowd_sim.envs.utils.human import Human


class CrowdNavNode(object):
    def __init__(self, args):
        if args.model_dir is not None:
            env_config_file = os.path.join(args.model_dir, os.path.basename(args.env_config))
            policy_config_file = os.path.join(args.model_dir, os.path.basename(args.policy_config))
            if args.il:
                model_weights = os.path.join(args.model_dir, 'il_model.pth')
            else:
                if os.path.exists(os.path.join(args.model_dir, 'resumed_rl_model.pth')):
                    model_weights = os.path.join(args.model_dir, 'resumed_rl_model.pth')
                else:
                    model_weights = os.path.join(args.model_dir, 'rl_model.pth')
        else:
            env_config_file = os.path.join(os.path.dirname(__file__), 'crowd_nav', args.env_config)
            policy_config_file = os.path.join(os.path.dirname(__file__), 'crowd_nav', args.policy_config)

        # configure logging and device
        logging.basicConfig(level=logging.INFO, format='%(asctime)s, %(levelname)s: %(message)s',
                            datefmt="%Y-%m-%d %H:%M:%S")
        device = torch.device("cuda:0" if torch.cuda.is_available() and args.gpu else "cpu")
        logging.info('Using device: %s', device)

        # configure policy
        self.policy = policy_factory[args.policy]()
        policy_config = configparser.RawConfigParser()
        policy_config.read(policy_config_file)
        self.policy.configure(policy_config)
        if self.policy.trainable:
            if args.model_dir is None:
                parser.error('Trainable policy must be specified with a model weights directory')
            self.policy.get_model().load_state_dict(torch.load(model_weights))

        # configure environment
        env_config = configparser.RawConfigParser()
        env_config.read(env_config_file)
        self.env = gym.make('CrowdSim-v0')
        self.env.configure(env_config)
        if args.square:
            self.env.test_sim = 'square_crossing'
        if args.circle:
            self.env.test_sim = 'circle_crossing'
        self.robot = Robot(env_config, 'robot')
        self.robot.set_policy(self.policy)
        self.env.set_robot(self.robot)
        self.explorer = Explorer(self.env, self.robot, device, gamma=0.9)

        self.policy.set_phase(args.phase)
        self.policy.set_device(device)
        # set safety space for ORCA in non-cooperative simulation
        if isinstance(self.robot.policy, ORCA):
            if self.robot.visible:
                self.robot.policy.safety_space = 0
            else:
                # because invisible case breaks the reciprocal assumption
                # adding some safety space improves ORCA performance. Tune this value based on your need.
                self.robot.policy.safety_space = 0
            logging.info('ORCA agent buffer: %f', self.robot.policy.safety_space)

        self.policy.set_env(self.env)
        self.robot.print_info()


        ################################ ROS implementation ################################ 
        self.robot_state = []
        self.finalgoal = None
        if self.robot.kinematics == 'holonomic':
            raise ValueError("The holonomic kinematics is not campatible with our robotic walker, please choose 'unicycle' agent model.")
        
        # ROS setup
        self.cmd_freq       = rospy.get_param("~cmd_freq", 5.0)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.2)
        self.sub_odom       = rospy.Subscriber("odom_filtered", Odometry, self.odom_cb, queue_size=1)
        self.sub_finalgoal  = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.finalgoal_cb, queue_size=1)
        self.pub_cmd        = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_vis        = rospy.Publisher('clustering_result', MarkerArray, queue_size=1)

        # Signal callback setup 
        rospy.on_shutdown(self.shutdown_cb)
        # Timer setup
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.cmd_freq), self.timer_cb)
        rospy.loginfo(rospy.get_name() + ' is ready.')
    

    def odom_cb(self, msg):
        euler_angle = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                            msg.pose.pose.orientation.y, 
                                            msg.pose.pose.orientation.z, 
                                            msg.pose.pose.orientation.w])
        self.robot_state = [msg.pose.pose.position.x, msg.pose.pose.position.y, euler_angle[2]]


    def finalgoal_cb(self, msg):
        rospy.loginfo("Get the finalgoal msg")
        if "odom" not in msg.header.frame_id:
            rospy.logerr("The finalgoal is not assigned in the global frame, skipping this assignment.")
        else:
            self.finalgoal = [msg.pose.position.x , msg.pose.position.y]


    def timer_cb(self, event):
        # Skip empty finalgoal
        if self.finalgoal is None:
            rospy.logwarn("Yet to set the finalgoal in rviz, skipping this timer callback.")
            return

        # Goal arrival situation
        dis_robot2goal = np.hypot(self.robot_state[0] - self.finalgoal[0], self.robot_state[1] - self.finalgoal[1])
        if dis_robot2goal <= self.goal_tolerance:
            self.finalgoal = None
            rospy.loginfo("goal reached! {:.2f}".format(dis_robot2goal))
            return
    
        # Pack observation list from message
        obseravation_list = []
        marker_array = MarkerArray()
        try:
            obseravation_msg = rospy.wait_for_message("rl_observation_array", Trk3DArray, timeout=(1.0 / self.cmd_freq))
            for obs_info in obseravation_msg.trks_list:
                # human = Human(env.config, 'humans')
                # human.set(obs_info.x, obs_info.y, 0, 0, 0)
                # obseravation_list.append([obs_info.x, obs_info.y, obs_info.vx, obs_info.vy, obs_info.radius])
                obseravation_list.append(ObservableState(obs_info.x, obs_info.y, obs_info.vx, obs_info.vy, obs_info.radius))

                # Visualization
                marker = Marker();
                marker.header.frame_id = "odom";
                marker.header.stamp = rospy.Time(0);
                marker.ns = "clustering_result";
                marker.id = len(marker_array.markers);
                marker.type = marker.CYLINDER;
                marker.lifetime = rospy.Duration(0.5);
                marker.action = marker.ADD;
                marker.pose.position.x = obs_info.x
                marker.pose.position.y = obs_info.y
                marker.pose.position.z = 1.0;
                marker.pose.orientation.w = 1.0;
                if obs_info.vx == 0.0 and obs_info.vy == 0.0:
                    marker.scale.x = 0.8 * 2;
                    marker.scale.y = 0.8 * 2;
                    marker.scale.z = 1.0;
                    marker.color.a = 0.4;
                    marker.color.g = 1.0;
                else:
                    marker.scale.x = 0.4 * 2;
                    marker.scale.y = 0.4 * 2;
                    marker.scale.z = 1.0;
                    marker.color.a = 0.4;
                    marker.color.r = 1.0;
                marker_array.markers.append(marker);
            self.pub_vis.publish(marker_array)

        except rospy.ROSException as e:
            rospy.logerr("Cannot get obseravation msg in timer interval {} sec(s), \
                            please check msg trasmission or fine tune timer interval.".format(1.0 / self.cmd_freq))
            self.timer.shutdown()
            rospy.signal_shutdown(str(e))

        self.robot.set(px=self.robot_state[0], py=self.robot_state[1], \
                    gx=self.finalgoal[0], gy=self.finalgoal[1], \
                    vx=0.0, vy=0.0, \
                    theta=self.robot_state[2], radius=0.4, \
                    v_pref=0.4)
        action = self.robot.act(obseravation_list)
        cmd_msg = Twist()
        cmd_msg.linear.x = action[0]
        cmd_msg.angular.z = action[1]
        self.pub_cmd.publish(cmd_msg)


    def shutdown_cb(self):
        self.pub_cmd.publish(Twist())
        rospy.loginfo("Shutdown " + rospy.get_name())


if __name__ == '__main__':
    rospy.init_node('sarl_node', anonymous=False)
    imp.reload(logging)

    parser = argparse.ArgumentParser('Parse configuration file')
    parser.add_argument('--env_config', type=str, default='configs/env.config')
    parser.add_argument('--policy_config', type=str, default='configs/policy.config')
    parser.add_argument('--policy', type=str, default='orca')
    parser.add_argument('--model_dir', type=str, default=None)
    parser.add_argument('--il', default=False, action='store_true')
    parser.add_argument('--gpu', default=False, action='store_true')
    parser.add_argument('--visualize', default=False, action='store_true')
    parser.add_argument('--phase', type=str, default='test')
    parser.add_argument('--test_case', type=int, default=None)
    parser.add_argument('--square', default=False, action='store_true')
    parser.add_argument('--circle', default=False, action='store_true')
    parser.add_argument('--video_file', type=str, default=None)
    parser.add_argument('--traj', default=False, action='store_true')
    parser.add_argument('__name', type=str, default='rosnode_name')     # Dummy args for ROS
    parser.add_argument('__log', type=str, default='log_file')      # Dummy args for ROS
    args = parser.parse_args()

    # CrowdNav instance
    node = CrowdNavNode(args)

    obseravation = node.env.reset(args.phase, args.test_case)
    done = False
    last_pos = np.array(node.robot.get_position())
    rospy.logwarn(last_pos)

    rospy.spin()

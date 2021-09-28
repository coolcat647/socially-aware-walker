#!/usr/bin/env python3

# import logging
import argparse
import configparser
import os
import sys
import torch
import numpy as np
import gym
import matplotlib.pyplot as plt
import imp
import copy

import rospy
import rospkg
import tf
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point, PointStamped, Pose2D
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path, Odometry
from walker_msgs.msg import Trk3DArray, Trk3D
from std_srvs.srv import Empty, EmptyResponse

from crowd_nav.utils.explorer import Explorer
from crowd_nav.policy.policy_factory import policy_factory
from crowd_sim.envs.utils.robot import Robot
from crowd_sim.envs.policy.orca import ORCA
from crowd_sim.envs.utils.state import ObservableState
from crowd_sim.envs.utils.human import Human


MAX_ANGULAR_VELOCITY    = 0.5
MAX_LINEAR_VELOCITY     = 0.5
ROBOT_RADIUS            = 0.7   # need to be consistent  with footprint approach
ROBOT_WHEELS_DISTANCE   = 0.6


def normalize_angle(theta):
    while theta >= np.pi * 2:
        theta -= np.pi * 2
    while theta < 0:
        theta += np.pi * 2
    return theta


def get_tf_matrix(theta, x=0.0, y=0.0):
    return np.array([[np.cos(theta), -np.sin(theta), x],
                     [np.sin(theta),  np.cos(theta), y],
                     [          0.0,            0.0, 1.0]], dtype=np.float32)


class CrowdNavNode(object):
    def __init__(self, args):
        if args.policy == "orca":
            env_config_file = os.path.join(os.path.dirname(__file__), 'crowd_nav', args.env_config)
            policy_config_file = os.path.join(os.path.dirname(__file__), 'crowd_nav', args.policy_config)
        elif args.model_dir is not None:
            if not os.path.isdir(args.model_dir):
                raise KeyError("model_dir is not found: {}".format(args.model_dir))

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
            raise KeyError("model_dir argument is not found")
            env_config_file = os.path.join(os.path.dirname(__file__), 'crowd_nav', args.env_config)
            policy_config_file = os.path.join(os.path.dirname(__file__), 'crowd_nav', args.policy_config)

        # configure logging and device
        # logging.basicConfig(level=logging.INFO, format='%(asctime)s, %(levelname)s: %(message)s',
        #                     datefmt="%Y-%m-%d %H:%M:%S")
        device = torch.device("cuda:0" if torch.cuda.is_available() and args.gpu else "cpu")
        # logging.info('Using device: %s', device)
        rospy.loginfo('Using device: %s', device)

        # configure policy
        # For LSTM h0 and c0 init
        torch.set_default_tensor_type(torch.cuda.FloatTensor if (torch.cuda.is_available() and args.gpu) else torch.FloatTensor)

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
                self.robot.policy.safety_space = 0.0
            else:
                # because invisible case breaks the reciprocal assumption
                # adding some safety space improves ORCA performance. Tune this value based on your need.
                self.robot.policy.safety_space = 0.0
            # logging.info('ORCA agent buffer: %f', self.robot.policy.safety_space)
            rospy.loginfo('ORCA agent buffer: %f', self.robot.policy.safety_space)

        self.policy.set_env(self.env)
        self.robot.print_info()

        # Custom transformation matrix
        self.tf_odom2sarl = np.ones((3, 3), dtype=np.float32)


        ################################ ROS implementation ################################
        self.robot_state = []
        self.finalgoal = None

        # ROS setup
        self.cmd_freq       = rospy.get_param("~cmd_freq", 5.0)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.4)
        self.sub_odom       = rospy.Subscriber("odom_filtered", Odometry, self.odom_cb, queue_size=1)
        self.sub_finalgoal  = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.finalgoal_cb, queue_size=1)
        self.pub_cmd        = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.pub_mrk_vis    = rospy.Publisher("clustering_result", MarkerArray, queue_size=1)
        self.pub_mrk_status = rospy.Publisher("robot_status", Marker, queue_size=1);
        self.pub_path_vis   = rospy.Publisher("path_vis", MarkerArray, queue_size=1)    # The same topic with path_fing_node
        self.srv_cancel     = rospy.Service("cancel_navigation", Empty, self.cancel_cb)
        self.static_obstacle_radius = rospy.get_param("static_obstacle_radius", 0.4)
        self.dynamic_obstacle_radius = rospy.get_param("dynamic_obstacle_radius", 0.4)
        rospy.set_param("navi_approach", args.policy.upper())

        # Robot status marker
        self.mrk_robot_status = Marker()
        self.mrk_array = MarkerArray()
        self.marker_init()      # Init function

        self.last_sarl_robot_state = None

        # Signal callback setup 
        rospy.on_shutdown(self.shutdown_cb)
        # Timer setup
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.cmd_freq), self.timer_cb)
        rospy.loginfo(rospy.get_name() + ' is ready.')


    def marker_init(self):
        mrk_goal = Marker()
        mrk_goal.header.frame_id = "odom";
        mrk_goal.ns = "subgoal";
        mrk_goal.type = mrk_goal.SPHERE;
        mrk_goal.action = mrk_goal.ADD;
        mrk_goal.pose.orientation.w = 1.0
        mrk_goal.scale.x = 0.4;
        mrk_goal.scale.y = 0.4;
        mrk_goal.scale.z = 0.4;
        mrk_goal.color.a = 0.0; # Set transparent at beginning
        mrk_goal.color.g = 1.0;
        mrk_goal.lifetime = rospy.Duration(30.0);
        mrk_goal.id = 0;
        self.mrk_array.markers.append(mrk_goal)

        self.mrk_robot_status.header.frame_id = "base_link";
        self.mrk_robot_status.ns = "robot_status";
        self.mrk_robot_status.type = self.mrk_robot_status.TEXT_VIEW_FACING;
        self.mrk_robot_status.action = self.mrk_robot_status.ADD;
        self.mrk_robot_status.pose.orientation.w = 1.0;
        self.mrk_robot_status.pose.position.z = 1.5;
        self.mrk_robot_status.scale.z = 0.4;
        self.mrk_robot_status.color.a = 1.0;
        self.mrk_robot_status.color.r = 1.0;
        self.mrk_robot_status.color.g = 1.0;
        self.mrk_robot_status.color.b = 1.0;
        self.mrk_robot_status.lifetime = rospy.Duration(8.0);


    def publish_robot_status_marker(self, str_message):
        self.mrk_robot_status.text = str_message;
        self.mrk_robot_status.header.stamp = rospy.Time();
        self.pub_mrk_status.publish(self.mrk_robot_status);


    def cancel_cb(self, req):
        self.finalgoal = None
        self.pub_cmd.publish(Twist())
        self.pub_cmd.publish(Twist())
        return EmptyResponse()


    def odom_cb(self, msg):
        # Get the yaw angle of robot
        euler_angle = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                            msg.pose.pose.orientation.y, 
                                            msg.pose.pose.orientation.z, 
                                            msg.pose.pose.orientation.w])
        self.robot_state = [msg.pose.pose.position.x, msg.pose.pose.position.y, euler_angle[2]]

        # robot_odom_matrix = get_tf_matrix(theta=euler_angle[2], x=msg.pose.pose.position.x, y=msg.pose.pose.position.y)
        # robot_sarl_matrix = np.dot(self.tf_odom2sarl, robot_odom_matrix)
        # self.robot_state = [robot_sarl_matrix[0, 2],
        #                     robot_sarl_matrix[1, 2],
        #                     np.arctan2(robot_sarl_matrix[1, 0], robot_sarl_matrix[0, 0])]



    def finalgoal_cb(self, msg):
        # Goal visualization
        self.mrk_array.markers[0].pose = msg.pose
        self.mrk_array.markers[0].color.a = 0.8;
        self.pub_path_vis.publish(self.mrk_array)
        self.publish_robot_status_marker("Get a new goal")

        rospy.loginfo("Get the finalgoal msg")
        if "odom" not in msg.header.frame_id:
            rospy.logerr("The finalgoal is not assigned in the global frame, aborting...")
            self.timer.shutdown()
            rospy.signal_shutdown("finalgoal frame is not correct.")
        else:
            self.finalgoal = [msg.pose.position.x, msg.pose.position.y]

            # Trick for SARL to reduce the timeout rate
            if not isinstance(self.robot.policy, ORCA) and self.finalgoal[0] == 0.0 and self.finalgoal[1] == 4.0:
                self.finalgoal[1] += 0.0

            tf_sarl2odom = get_tf_matrix(theta=np.arctan2(msg.pose.position.y - self.robot_state[1],
                                                     msg.pose.position.x - self.robot_state[0]) - np.pi / 2,
                                         x=(msg.pose.position.x + self.robot_state[0]) / 2,
                                         y=(msg.pose.position.y + self.robot_state[1]) / 2)
            self.tf_odom2sarl = np.linalg.inv(tf_sarl2odom)


            # transform current position and goal position to sarl frame
            odom_start_matrix = get_tf_matrix(theta=self.robot_state[2], x=self.robot_state[0], y=self.robot_state[1])
            sarl_start_matrix = np.dot(self.tf_odom2sarl, odom_start_matrix)

            robot_goal_matrix = get_tf_matrix(theta=0.0, x=self.finalgoal[0], y=self.finalgoal[1])
            sarl_goal_matrix = np.dot(self.tf_odom2sarl, robot_goal_matrix)
            # print("\nodom_start_matrix\n", odom_start_matrix)
            # print("\nsarl_start_matrix\n", sarl_start_matrix)
            # print("\nrobot_goal_matrix\n", robot_goal_matrix)
            # print("\nsarl_goal_matrix\n", sarl_goal_matrix)

            self.robot.set(px=sarl_start_matrix[0, 2], py=sarl_start_matrix[1, 2],
                           gx=sarl_goal_matrix[0, 2], gy=sarl_goal_matrix[1, 2],
                           vx=0.0, vy=0.0,
                           theta=np.arctan2(sarl_start_matrix[1, 0], sarl_start_matrix[0, 0]),
                           radius=ROBOT_RADIUS,
                           v_pref=MAX_LINEAR_VELOCITY)

            # USE RAW COORDINATE
            # self.robot.set(px=self.robot_state[0], py=self.robot_state[1],
            #                gx=msg.pose.position.x, gy=msg.pose.position.y,
            #                vx=0.0, vy=0.0,
            #                theta=np.arctan2(sarl_start_matrix[1, 0], sarl_start_matrix[0, 0]), radius=0.4,
            #                v_pref=MAX_LINEAR_VELOCITY)
            rospy.loginfo("current position: ({:.2f}, {:.2f})".format(self.robot.px, self.robot.py))
            rospy.loginfo("goal position: ({:.2f}, {:.2f})".format(self.robot.gx, self.robot.gy))


    def pack_marker_array(self, marker_array: Trk3DArray, obs_info: Trk3D):
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
            marker.scale.x = self.static_obstacle_radius * 2;
            marker.scale.y = self.static_obstacle_radius * 2;
            marker.scale.z = 1.0;
            marker.color.a = 0.4;
            marker.color.g = 1.0;
        else:
            marker.scale.x = self.dynamic_obstacle_radius * 2;
            marker.scale.y = self.dynamic_obstacle_radius * 2;
            marker.scale.z = 1.0;
            marker.color.a = 0.4;
            marker.color.r = 1.0;
        marker_array.markers.append(marker);


    def timer_cb(self, event):
        # Skip empty finalgoal
        if self.finalgoal is None:
            rospy.logwarn("Yet to set the finalgoal in rviz.")
            return
        elif len(self.robot_state) == 0:
            rospy.logwarn("Cannot get robot odom info, skipping this timer callback.")
            return

        # Goal arrival situation
        dis_robot2goal = np.hypot(self.robot_state[0] - self.finalgoal[0], self.robot_state[1] - self.finalgoal[1])
        if dis_robot2goal < (self.goal_tolerance - 0.02):
            self.finalgoal = None
            rospy.loginfo("goal reached! {:.2f}".format(dis_robot2goal))
            self.pub_cmd.publish(Twist())
            rospy.sleep(0.5)
            return
    
        # Pack observation list from message
        obseravation_list = []
        marker_array = MarkerArray()

        odom_start_matrix = get_tf_matrix(theta=self.robot_state[2], x=self.robot_state[0], y=self.robot_state[1])
        sarl_start_matrix = np.dot(self.tf_odom2sarl, odom_start_matrix)
        sarl_robot_state = [sarl_start_matrix[0, 2], sarl_start_matrix[1, 2], np.arctan2(sarl_start_matrix[1, 0], sarl_start_matrix[0, 0])]

        try:
            obseravation_msg = rospy.wait_for_message("rl_observation_array", Trk3DArray, timeout=(1.0 / self.cmd_freq))
        except rospy.ROSException as e:
            obseravation_msg = Trk3DArray()
            rospy.logwarn("Cannot get obseravation msg {} sec(s), use the fake msg.".format(1.0 / self.cmd_freq))

        # Collect observations list
        for obs_info in obseravation_msg.trks_list:
            obs_direction = np.arctan2(obs_info.vy, obs_info.vx)
            obs_speed = np.hypot(obs_info.vx, obs_info.vy)
            obstacle_radius = self.dynamic_obstacle_radius if obs_speed > 0.25 else self.static_obstacle_radius
            odom_obs_matrix = get_tf_matrix(theta=obs_direction, x=obs_info.x, y=obs_info.y)
            sarl_obs_matrix = np.dot(self.tf_odom2sarl, odom_obs_matrix)
            sarl_obs_direction = np.arctan2(sarl_obs_matrix[1, 0], sarl_obs_matrix[0, 0])
            obseravation_list.append(ObservableState(sarl_obs_matrix[0, 2],
                                                     sarl_obs_matrix[1, 2],
                                                     obs_speed * np.cos(sarl_obs_direction),
                                                     obs_speed * np.sin(sarl_obs_direction),
                                                     obstacle_radius))
            # USE RAW COORDINATE
            # obseravation_list.append(ObservableState(obs_info.x, obs_info.y, obs_info.vx, obs_info.vy, obs_info.radius))

            # Visualization
            self.pack_marker_array(marker_array, obs_info)

        # If there are no any observations, create a fake one
        if len(obseravation_list) == 0:
            obseravation_list.append(ObservableState(10, 10, 0, 0, 0.5))

        # Feed the observations to the agent
        action = self.robot.act(obseravation_list)
        # print(action)
        dummy_ob, _, done, info = self.env.step(action)

        cmd_msg = Twist()
        # ORCA (holonomic action) -> (vx, vy)
        if self.robot.kinematics == 'holonomic':
            desired_theta = np.arctan2(action[1], action[0])

            # Deal with high rotation angle situation
            if np.abs(desired_theta - sarl_robot_state[2]) > np.pi / 4:
                diff_angle = normalize_angle(desired_theta - sarl_robot_state[2])
                if diff_angle < 0 or diff_angle > np.pi:  cmd_msg.angular.z = -MAX_ANGULAR_VELOCITY
                else: cmd_msg.angular.z = MAX_ANGULAR_VELOCITY
                cmd_msg.linear.x = np.abs(cmd_msg.angular.z) * ROBOT_WHEELS_DISTANCE / 2
            else:
                cmd_msg.angular.z = np.clip(desired_theta - sarl_robot_state[2],
                                            -MAX_ANGULAR_VELOCITY,
                                            MAX_ANGULAR_VELOCITY)
                cmd_msg.linear.x = np.hypot(action[0], action[1])

            # Predict the robot safety after 0.25 and 0.5 seconds, respectively
            danger_candidates =[]
            for ob in obseravation_list:
                if np.hypot(ob.px - sarl_robot_state[0], ob.py - sarl_robot_state[1]) < (ob.radius + ROBOT_RADIUS * 2):
                    danger_candidates.append(ob)
            flag_danger = False
            for delta_t in [0.1, 0.2, 0.3, 0.4, 0.5]:
                robot_theta = sarl_robot_state[2] + cmd_msg.angular.z * delta_t
                robot_x = sarl_robot_state[0] + np.cos(robot_theta) * cmd_msg.linear.x * delta_t
                robot_y = sarl_robot_state[1] + np.sin(robot_theta) * cmd_msg.linear.x * delta_t
                for danger_ob in danger_candidates:
                    ob_x = ob.px + ob.vx * delta_t
                    ob_y = ob.py + ob.vy * delta_t
                    if np.hypot(ob_x - robot_x, ob_y - robot_y) < (ob.radius + ROBOT_RADIUS):
                        flag_danger = True
                        break
                if flag_danger: break

            if flag_danger:
                self.publish_robot_status_marker("Get danger, stopping")
                rospy.loginfo("Get danger due to simulated-holonomic motion, stopping...")
                cmd_msg = Twist()

        else:
            # LSTM-RL, SARL (unicycle kinematic model) action -> (v, w)
            cmd_msg.linear.x = action[0]
            cmd_msg.angular.z = action[1]
        self.pub_cmd.publish(cmd_msg)
        self.robot.set_position([sarl_robot_state[0], sarl_robot_state[1]])

        if self.last_sarl_robot_state is None:
            self.last_sarl_robot_state = copy.deepcopy(sarl_robot_state)
        else:
            sarl_robot_vx = sarl_robot_state[0] - self.last_sarl_robot_state[0]
            sarl_robot_vy = sarl_robot_state[1] - self.last_sarl_robot_state[1]
            self.robot.set_velocity([sarl_robot_vx, sarl_robot_vy])

        # Publish visualization msg if there are any subscribers existed
        if self.pub_mrk_vis.get_num_connections() > 0:
            self.pub_mrk_vis.publish(marker_array)


    def shutdown_cb(self):
        self.pub_cmd.publish(Twist())
        rospy.loginfo("Shutdown " + rospy.get_name())


if __name__ == '__main__':
    rospy.init_node('sarl_node', anonymous=False)
    # imp.reload(logging)

    parser = argparse.ArgumentParser('Parse configuration file')
    parser.add_argument('--env_config', type=str, default='configs/env.config')
    parser.add_argument('--policy_config', type=str, default='configs/policy.config')
    parser.add_argument('--policy', type=str, default='orca')
    parser.add_argument('--model_dir', type=str, default=None)
    parser.add_argument('--il', default=False, action='store_true')
    parser.add_argument('--gpu', default=True, action='store_true')
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

    rospy.spin()

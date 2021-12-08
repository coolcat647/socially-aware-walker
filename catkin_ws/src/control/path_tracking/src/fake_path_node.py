#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point, PointStamped, Pose2D
from nav_msgs.msg import Path, Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

path_step_size = 0.6

if __name__ == '__main__':
    rospy.init_node('steering_control_with_user_pushing_node', anonymous=False)

    pub_path = rospy.Publisher("walker/walkable_path", Path, queue_size=1)
    rospy.sleep(1)

    path_msg = Path()
    path_length = 0


    '''
        straight line
    '''
    # while path_length < 20:
    #     pose_msg = PoseStamped()
    #     pose_msg.header.stamp = rospy.Time.now()
    #     pose_msg.header.frame_id = "odom"
    #     pose_msg.pose.position.x = path_length
    #     pose_msg.pose.position.y = 0.4 + path_length * 0.5
    #     if len(path_msg.poses) == 0:
    #         pose_msg.pose.orientation.w = 1
    #     else:
    #         q = quaternion_from_euler(0, 0, np.arctan2(pose_msg.pose.position.y - path_msg.poses[-1].pose.position.y, 
    #                                                     pose_msg.pose.position.x - path_msg.poses[-1].pose.position.x))
    #         pose_msg.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    #     path_msg.poses.append(pose_msg)
    #     path_length += path_step_size


    '''
        90 degree turning
    '''
    # while path_length < 20:
    #     pose_msg = PoseStamped()
    #     pose_msg.header.stamp = rospy.Time.now()
    #     pose_msg.header.frame_id = "odom"

    #     if path_length < path_step_size * 5:
    #         pose_msg.pose.position.x = path_length
    #         pose_msg.pose.position.y = 0.0
    #     else:
    #         pose_msg.pose.position.x = path_step_size * 5
    #         pose_msg.pose.position.y = path_length - path_step_size * 5

    #     if len(path_msg.poses) == 0:
    #         pose_msg.pose.orientation.w = 1
    #     else:
    #         q = quaternion_from_euler(0, 0, np.arctan2(pose_msg.pose.position.y - path_msg.poses[-1].pose.position.y, 
    #                                                     pose_msg.pose.position.x - path_msg.poses[-1].pose.position.x))
    #         pose_msg.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    #     path_msg.poses.append(pose_msg)
    #     path_length += path_step_size


    '''
        figure-eight curve
    '''
    SHAPE_SCALE = 5.0
    t = np.linspace(np.pi * 2 / 50, np.pi * 98 / 50, 50)
    x = np.sin(t) * SHAPE_SCALE
    y = np.sin(t) * np.cos(t) * SHAPE_SCALE
    for i in range(t.shape[0]):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.position.x = x[i]
        pose_msg.pose.position.y = y[i]
        if i == 0:
            pose_msg.pose.orientation.w = 1
        else:
            q = quaternion_from_euler(0, 0, np.arctan2(y[i] - y[i - 1], x[i] - x[i - 1])) 
            pose_msg.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        path_msg.poses.append(pose_msg)
        path_length += path_step_size


    '''
        S-shape curve
    '''
    # SHAPE_SCALE = 2.0
    # t1 = np.linspace(np.pi / 2, 0 + np.pi / 2 / 5, 4)
    # x = np.cos(t1) * SHAPE_SCALE
    # y = np.sin(t1) * SHAPE_SCALE - 2.0
    # t2 = np.linspace(np.pi, np.pi * 3 / 2, 5)
    # x = np.append(x, np.cos(t2) * SHAPE_SCALE + 4.0)
    # y = np.append(y, np.sin(t2) * SHAPE_SCALE - 2.0)
    # for i in range(x.shape[0]):
    #     pose_msg = PoseStamped()
    #     pose_msg.header.stamp = rospy.Time.now()
    #     pose_msg.header.frame_id = "odom"
    #     pose_msg.pose.position.x = x[i]
    #     pose_msg.pose.position.y = y[i]
    #     if i == 0:
    #         pose_msg.pose.orientation.w = 1
    #     else:
    #         q = quaternion_from_euler(0, 0, np.arctan2(y[i] - y[i - 1], x[i] - x[i - 1])) 
    #         pose_msg.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
    #     path_msg.poses.append(pose_msg)
    #     path_length += path_step_size


    path_msg.poses.reverse()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = "odom"
    pub_path.publish(path_msg)
    rospy.loginfo("path is published")
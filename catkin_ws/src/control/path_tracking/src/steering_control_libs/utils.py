# -*- coding: utf-8 -*-

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D, Twist

# Normalize an angle to [-pi, pi].
def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi
    while angle < -np.pi:
        angle += 2.0 * np.pi
    return angle


# Proportional control
def proportional_control(target_value, current_value, gain):
    return (target_value - current_value) * gain


# Compute index in the trajectory list of the target.
def calc_target_index(robot_pose, target_path, robot_ref_length):
    """
    :param robot_pose: (Pose2D)
    :param target_path: ([Pose2D])
    :param robot_ref_length: (float)
    :return: (int, float)
    """
    if not isinstance(robot_pose, Pose2D):
        raise NotImplementedError("Variable \'robot_pose\' should be Pose2D.")
    robot_theta = robot_pose.theta

    # Calc front axle position
    fx = robot_pose.x + robot_ref_length * np.cos(robot_theta)
    fy = robot_pose.y + robot_ref_length * np.sin(robot_theta)

    # Search nearest point index
    dx = []
    dy = []
    for tmp_pose in target_path:
        dx.append(fx - tmp_pose.x)
        dy.append(fy - tmp_pose.y)
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)
    # print("target_idx:", target_idx)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(robot_theta + np.pi / 2),
                      -np.sin(robot_theta + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


def calc_target_index_short(robot_pose, target_path, robot_ref_length):
    if not isinstance(robot_pose, Pose2D):
        raise NotImplementedError("Variable \'robot_pose\' should be Pose2D.")
    robot_theta = robot_pose.theta

    # Calc front axle position
    fx = robot_pose.x + robot_ref_length * np.cos(robot_theta) * 0.5
    fy = robot_pose.y + robot_ref_length * np.sin(robot_theta) * 0.5

    # Search nearest point index
    dx = []
    dy = []
    for tmp_pose in target_path:
        dx.append(fx - tmp_pose.x)
        dy.append(fy - tmp_pose.y)
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)
    # print("target_idx:", target_idx)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(robot_theta + np.pi / 2),
                      -np.sin(robot_theta + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


# Stanley steering control
def stanley_control(robot_pose, robot_twist, target_path, robot_ref_length, crosstrack_error_gain=5.0):
    """
    :param robot_pose: (Pose2D)
    :param robot_twist: (Twist)
    :param target_path: ([Pose2D])
    :param robot_ref_length: (float)
    :param crosstrack_error_gain: (float)
    :return: (float, int)
    """
    if not isinstance(robot_pose, Pose2D) or not isinstance(robot_twist, Twist):
        raise NotImplementedError("Variables \'robot_pose\' should be Pose2D and \'robot_twist\' should be Twist.")

    current_target_idx, error_front_axle = calc_target_index(robot_pose, target_path, robot_ref_length)

    # heading error
    heading_error = normalize_angle(target_path[current_target_idx].theta - robot_pose.theta)
    # crosstrack_error: is defined as the lateral distance between the heading vector and the target point as follows.
    crosstrack_error = np.arctan2(crosstrack_error_gain * error_front_axle, robot_twist.linear.x)
    # total steering error
    total_steering_error = heading_error + crosstrack_error

    return total_steering_error, current_target_idx


# Steering control with only heading error
def heading_control(robot_pose, robot_twist, target_path, robot_ref_length):
    """
    :param robot_pose: (Pose2D)
    :param robot_twist: (Twist)
    :param target_path: ([Pose2D])
    :param robot_ref_length: (float)
    :param crosstrack_error_gain: (float)
    :return: (float, int)
    """
    if not isinstance(robot_pose, Pose2D) or not isinstance(robot_twist, Twist):
        raise NotImplementedError("Variables \'robot_pose\' should be Pose2D and \'robot_twist\' should be Twist.")

    current_target_idx, error_front_axle = calc_target_index_short(robot_pose, target_path, robot_ref_length)

    # heading error
    heading_error = normalize_angle(target_path[current_target_idx].theta - robot_pose.theta)

    return heading_error, current_target_idx


# My steering control
def my_steering_control(robot_pose, robot_twist, target_path, robot_ref_length):
    """
    :param robot_pose: (Pose2D)
    :param robot_twist: (Twist)
    :param target_path: ([Pose2D])
    :param robot_ref_length: (float)
    :return: (float, int)
    """
    if not isinstance(robot_pose, Pose2D) or not isinstance(robot_twist, Twist):
        raise NotImplementedError("Variables \'robot_pose\' should be Pose2D and \'robot_twist\' should be Twist.")

    current_target_idx, error_front_axle = calc_target_index(robot_pose, target_path, robot_ref_length)

    # heading error
    heading_error = normalize_angle(target_path[current_target_idx].theta - robot_pose.theta)

    # total steering error
    total_steering_error = np.abs(robot_twist.linear.x) * heading_error + error_front_axle * robot_twist.linear.x

    return total_steering_error, current_target_idx
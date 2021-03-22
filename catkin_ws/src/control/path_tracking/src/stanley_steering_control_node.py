#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import threading
import numpy as np
import matplotlib.pyplot as plt
import sys
import time
import copy
from CubicSpline import cubic_spline_planner

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point, PointStamped, Pose2D
from nav_msgs.msg import Path, Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Float32

SPEED_PROPORTIONAL_GAIN     = 1.0   # speed proportional gain
CROSSTRACK_ERROR_GAIN       = 5.0   # crosstrack error gain
LENGTH_OF_ROBOT_BASE        = 0.6   # [m] Wheel base of vehicle
MAX_ANGULAR_VELOCITY = 1.0
MAX_LINEAR_SPEED = 0.4

class StanleyControlNode(object):
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)

        # Path related
        self.flat_path = []
        self.updated_flat_path = []

        self.flag_path_update = False

        # Odom related
        self.robot_pose = Pose2D()
        self.robot_twist = Twist()

        # ROS parameters
        self.map_resolution         = rospy.get_param("~map_resolution", 0.2)
        self.smooth_path_resolution = self.map_resolution / 2.0     # much smoother than original path
        self.cmd_freq               = rospy.get_param("~cmd_freq", 5.0)
        self.goal_tolerance         = rospy.get_param("~goal_tolerance", 0.2)
        

        # ROS publisher & subscriber
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_path_flat = rospy.Publisher('smooth_path', Path, queue_size=1)
        self.pub_tracking_progress = rospy.Publisher('tracking_progress', Float32, queue_size=1)
        self.pub_short_term_goal = rospy.Publisher('short_term_goal', PointStamped, queue_size=1)

        self.sub_path = rospy.Subscriber("walkable_path", Path, self.path_cb, queue_size=1)
        self.sub_odom = rospy.Subscriber("odom_filtered", Odometry, self.odom_cb, queue_size=1)

        rospy.loginfo(rospy.get_name() + ' is ready.')
        

    def odom_cb(self, msg):
        self.robot_pose.x = msg.pose.pose.position.x
        self.robot_pose.y = msg.pose.pose.position.y
        euler_angle = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                            msg.pose.pose.orientation.y, 
                                            msg.pose.pose.orientation.z, 
                                            msg.pose.pose.orientation.w])
        self.robot_pose.theta = euler_angle[2]
        self.robot_twist = msg.twist.twist


    def path_cb(self, msg):
        cx = np.array([], dtype=np.float32)
        cy = np.array([], dtype=np.float32)

        if len(msg.poses) == 0:
            self.updated_flat_path = []
            self.flag_path_update = True
        elif len(msg.poses) < 3:
            yaw = np.arctan2(msg.poses[0].pose.position.y - self.robot_pose.y, msg.poses[0].pose.position.x - self.robot_pose.x)
            self.updated_flat_path = [Pose2D(x=msg.poses[0].pose.position.x, y=msg.poses[0].pose.position.y, theta=yaw),]
            self.flag_path_update = True

        else:
            path_x_raw = []
            path_y_raw = []
            for i in range(len(msg.poses)-1, -1, -1):
                path_x_raw.append(msg.poses[i].pose.position.x)
                path_y_raw.append(msg.poses[i].pose.position.y)
            cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(path_x_raw,
                                                                          path_y_raw,
                                                                          ds=self.smooth_path_resolution)
            self.updated_flat_path = []
            for i in range(len(cx)):
                self.updated_flat_path.append(Pose2D(x=cx[i], y=cy[i], theta=cyaw[i]))
            self.flag_path_update = True

        # Visualiztion
        flat_path_msg = Path()
        for i in range(len(cx)):
            tmp_pose = PoseStamped()
            tmp_pose.pose.position = Point(cx[i], cy[i], 0)
            q = quaternion_from_euler(0, 0, cyaw[i])
            tmp_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            flat_path_msg.poses.append(tmp_pose)
        flat_path_msg.header.frame_id = msg.header.frame_id
        flat_path_msg.header.stamp = rospy.Time.now()
        self.pub_path_flat.publish(flat_path_msg)


    def pid_control(self, target_value, current_value):
        return SPEED_PROPORTIONAL_GAIN * (target_value - current_value)


    def stanley_control(self, robot_pose, robot_twist, target_path, last_target_idx):
        """
        Stanley steering control.

        :param robot_pose: (Pose2D)
        :param target_path: ([Pose2D])
        :param last_target_idx: (int)
        :return: (float, int)
        """
        current_target_idx, error_front_axle = self.calc_target_index(robot_pose, target_path)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # heading error
        heading_error = self.normalize_angle(target_path[current_target_idx].theta - robot_pose.theta)

        # crosstrack_error: is defined as the lateral distance between the heading vector and the target point as follows.
        crosstrack_error = np.arctan2(CROSSTRACK_ERROR_GAIN * error_front_axle, robot_twist.linear.x + 0.1) ########################## TODO: Check

        # Steering control
        total_steering_error = heading_error + crosstrack_error

        return total_steering_error, current_target_idx, heading_error


    def normalize_angle(self, angle):
        # Normalize an angle to [-pi, pi].
        while angle > np.pi:
            angle -= 2.0 * np.pi

        while angle < -np.pi:
            angle += 2.0 * np.pi

        return angle


    def calc_target_index(self, robot_pose, target_path):      
        # Compute index in the trajectory list of the target.

        # Calc front axle position
        # fx = robot_pose.x + LENGTH_OF_ROBOT_BASE * 2 * np.cos(robot_pose.theta)
        # fy = robot_pose.y + LENGTH_OF_ROBOT_BASE * 2 * np.sin(robot_pose.theta)
        fx = robot_pose.x + LENGTH_OF_ROBOT_BASE * np.cos(robot_pose.theta)
        fy = robot_pose.y + LENGTH_OF_ROBOT_BASE * np.sin(robot_pose.theta)

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
        front_axle_vec = [-np.cos(robot_pose.theta + np.pi / 2),
                          -np.sin(robot_pose.theta + np.pi / 2)]
        error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

        return target_idx, error_front_axle


    def shutdown_cb(self):
        self.pub_cmd.publish(Twist())
        rospy.loginfo("Shutdown " + rospy.get_name())
        

if __name__ == '__main__':
    rospy.init_node('stanley_control_node', anonymous=False)
    node = StanleyControlNode()

    rate = rospy.Rate(node.cmd_freq)
    while not rospy.is_shutdown():
        if node.flag_path_update == True:
            node.flag_path_update = False

            cmp_result = all(map(lambda x, y: x == y, node.flat_path, node.updated_flat_path))
            if not cmp_result or len(node.flat_path) != len(node.updated_flat_path):
                node.flat_path = copy.deepcopy(node.updated_flat_path)
                # rospy.loginfo("path updated!")

        if len(node.flat_path) == 0:
            rospy.loginfo("Empty planning path, wait for new path")
            node.pub_cmd.publish(Twist())
        else:
            # Steering control law
            target_idx, _  = node.calc_target_index(node.robot_pose, node.flat_path)            
            total_steering_error, target_idx, heading_error = node.stanley_control(node.robot_pose, node.robot_twist, node.flat_path, target_idx)
            
            # Publish tracking progress
            tracking_progress = (target_idx + 1) / len(node.flat_path)
            node.pub_tracking_progress.publish(tracking_progress)

            # Publish short term goal
            point_msg = PointStamped()
            point_msg.point =  Point(node.flat_path[target_idx].x, node.flat_path[target_idx].y, 0)
            point_msg.header.stamp = rospy.Time()
            point_msg.header.frame_id = "odom"
            node.pub_short_term_goal.publish(point_msg)

            # if tracking_progress < 1.0:
            dis_robot2goal = np.hypot(node.robot_pose.x - node.flat_path[-1].x, node.robot_pose.y - node.flat_path[-1].y)
            # if dis_robot2goal > node.map_resolution :
            if dis_robot2goal > node.goal_tolerance:
                # Car command 
                dt = 1.0 / node.cmd_freq
                cmd_msg = Twist()

                # Assign angular velocity command
                cmd_msg.angular.z = np.clip(total_steering_error * dt, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY)
                
                # Assign linear velocity command
                if np.abs(total_steering_error) >= np.pi / 4:
                    target_speed = np.abs(cmd_msg.angular.z) * 0.6 / 2
                else:
                    target_speed = MAX_LINEAR_SPEED
                accel_linear = node.pid_control(target_speed, node.robot_twist.linear.x)
                cmd_msg.linear.x = np.clip(node.robot_twist.linear.x + accel_linear * dt, np.abs(cmd_msg.angular.z) * 0.6 / 2, MAX_LINEAR_SPEED)
 
                node.pub_cmd.publish(cmd_msg)
            else:
                rospy.loginfo("goal reached! {:.2f}".format(dis_robot2goal))
                node.pub_cmd.publish(Twist())
                node.pub_tracking_progress.publish(1.0)

        rate.sleep()


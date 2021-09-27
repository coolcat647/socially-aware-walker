#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import copy
from steering_control_libs import cubic_spline_planner
from steering_control_libs.utils import calc_target_index, stanley_control, proportional_control, my_steering_control

# ROS
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion, Point, PointStamped, Pose2D, Wrench, WrenchStamped
from nav_msgs.msg import Path, Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Float32
import message_filters

SPEED_PROPORTIONAL_GAIN     = 2.0   # speed proportional gain
CROSSTRACK_ERROR_GAIN       = 2.5   # crosstrack error gain

ROBOT_REF_LENGTH            = 0.6   # [m] Wheel base of vehicle
ROBOT_WHEELS_DISTANCE       = 0.6

class SteeringControlWithPushingNode(object):
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
        self.cmd_freq               = rospy.get_param("~cmd_freq", 10.0)
        self.goal_tolerance         = rospy.get_param("~goal_tolerance", 0.2)
        self.robot_constraints_dict = rospy.get_param('constraints')
        self.robot_dynamics_dict    = rospy.get_param('dynamics')

        # ROS publisher & subscriber
        self.pub_cmd = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_path_flat = rospy.Publisher('smooth_path', Path, queue_size=1)
        self.pub_tracking_progress = rospy.Publisher('tracking_progress', Float32, queue_size=1)
        self.pub_short_term_goal = rospy.Publisher('short_term_goal', PointStamped, queue_size=1)

        self.sub_path = rospy.Subscriber("walkable_path", Path, self.path_cb, queue_size=1)
        sub_user_force = message_filters.Subscriber("force_filtered", WrenchStamped)
        # sub_user_cmd = message_filters.Subscriber("user_contributed/cmd_vel", Twist)
        sub_odom = message_filters.Subscriber("odom_filtered", Odometry)
        ts = message_filters.ApproximateTimeSynchronizer([sub_user_force, sub_odom], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.cmd_odom_cb)


        self.pub_accel = rospy.Publisher('accel', Twist, queue_size=1)
        self.pub_inhibition_force = rospy.Publisher('inhibition_force', Float32, queue_size=1)
        self.user_force_msg = Wrench()
        self.last_v = 0
        self.last_w = 0
        self.last_time = None

        rospy.loginfo(rospy.get_name() + ' is ready.')
    

    def cmd_odom_cb(self, user_force_msg, odom_msg):
        # Publish acceleration
        if self.last_time is not None:
            accel_msg = Twist()
            dt = (odom_msg.header.stamp - self.last_time).to_sec()
            accel_msg.linear.x = (odom_msg.twist.twist.linear.x - self.robot_twist.linear.x) / dt
            accel_msg.angular.z = (odom_msg.twist.twist.angular.z - self.robot_twist.angular.z) / dt
            if self.pub_accel.get_num_connections() > 0:
                self.pub_accel.publish(accel_msg)
        self.last_time = odom_msg.header.stamp

        self.robot_pose.x = odom_msg.pose.pose.position.x
        self.robot_pose.y = odom_msg.pose.pose.position.y
        euler_angle = euler_from_quaternion([odom_msg.pose.pose.orientation.x, 
                                            odom_msg.pose.pose.orientation.y, 
                                            odom_msg.pose.pose.orientation.z, 
                                            odom_msg.pose.pose.orientation.w])
        self.robot_pose.theta = euler_angle[2]
        self.robot_twist = odom_msg.twist.twist
        # self.robot_twist.angular.z = odom_msg.twist.twist.angular.z
        self.user_force_msg = user_force_msg.wrench


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


    def shutdown_cb(self):
        self.pub_cmd.publish(Twist())
        rospy.loginfo("Shutdown " + rospy.get_name())


if __name__ == '__main__':
    rospy.init_node('steering_control_with_user_pushing_node', anonymous=False)
    node = SteeringControlWithPushingNode()

    # Control rate
    rate = rospy.Rate(node.cmd_freq)

    flag_message_published = False
    flag_high_steering_error = False

    while not rospy.is_shutdown():
        if node.flag_path_update == True:
            node.flag_path_update = False

            # Check whether the updated flat path is equal to the old path
            cmp_result = all(map(lambda x, y: x == y, node.flat_path, node.updated_flat_path))
            if not cmp_result or len(node.flat_path) != len(node.updated_flat_path):
                node.flat_path = copy.deepcopy(node.updated_flat_path)
                # rospy.loginfo("path updated!")

        if len(node.flat_path) == 0:
            if not flag_message_published: 
                rospy.loginfo("Empty planning path, wait for new path")
                flag_message_published = True
            # Stop the robot
            cmd_msg = Twist()
            dt = 1.0 / node.cmd_freq
            accel_linear = proportional_control(0, node.robot_twist.linear.x, SPEED_PROPORTIONAL_GAIN)
            cmd_msg.linear.x = np.clip(node.robot_twist.linear.x + accel_linear * dt,
                                        0.0,
                                        node.robot_constraints_dict["max_linear_velocity"])
            node.pub_cmd.publish(cmd_msg)

        else:
            flag_message_published = False

            # Steering control law
            # total_steering_error, target_idx = stanley_control(node.robot_pose, 
            #                                                     node.robot_twist, 
            #                                                     node.flat_path, 
            #                                                     ROBOT_REF_LENGTH,
            #                                                     CROSSTRACK_ERROR_GAIN)
            total_steering_error, target_idx = my_steering_control(node.robot_pose,
                                                                node.robot_twist,
                                                                node.flat_path,
                                                                ROBOT_REF_LENGTH,
                                                                CROSSTRACK_ERROR_GAIN)
            
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
            
            now_time = rospy.Time().now()

            if dis_robot2goal > node.goal_tolerance:
                # Car command 
                dt = 1.0 / node.cmd_freq
                cmd_msg = Twist()

                # Assign angular velocity command
                '''
                Notice:
                    total_steering_error = -(theta - theta_desired)
                    theta_dot_dot = -k2 * theta_dot + k3 * total_steering_error
                '''
                if node.robot_twist.linear.x > 0.1:
                    accel_angular = -node.robot_twist.angular.z * 1 + total_steering_error * 1
                    cmd_msg.angular.z = np.clip(node.robot_twist.angular.z + accel_angular * dt,
                                                -node.robot_constraints_dict["max_angular_velocity"], 
                                                node.robot_constraints_dict["max_angular_velocity"])
                
                # Assign linear velocity command
                force_y = node.user_force_msg.force.y if np.abs(node.user_force_msg.force.y) > node.robot_constraints_dict["min_enable_force"] else 0.0
                total_mass = node.robot_dynamics_dict["mass"] + (-node.user_force_msg.force.z / 9.8)       # Newton -> kg

                x_t = np.abs(total_steering_error) * 2 / np.pi
                inhibition_force = (force_y / total_mass) * (1 / (1 + np.exp(-x_t * 10 + 5)))
                node.pub_inhibition_force.publish(inhibition_force * 30)
                v_dot = force_y / total_mass - \
                        node.last_v * node.robot_dynamics_dict["damping_xy"] / total_mass - \
                        node.robot_dynamics_dict["constant_fraction_xy"] - \
                        inhibition_force
                # next_v = np.clip(node.last_v + v_dot * dt, 0.0, node.robot_constraints_dict["max_linear_velocity"])
                next_v = node.last_v + v_dot * dt
                # print("e: {:.2f} degs".format(total_steering_error * 180 / np.pi))
                # print("x_t: {:.2f} inh_force: {:.2f}".format(x_t, inhibition_force))

                target_speed = node.last_v
                accel_linear = proportional_control(target_speed, node.robot_twist.linear.x, SPEED_PROPORTIONAL_GAIN)
                # cmd_msg.linear.x = np.clip(node.robot_twist.linear.x + accel_linear * dt,
                #                             np.abs(cmd_msg.angular.z) * ROBOT_WHEELS_DISTANCE / 2,
                #                             node.robot_constraints_dict["max_linear_velocity"])
                cmd_msg.linear.x = np.clip(node.robot_twist.linear.x + accel_linear * dt,
                                            -node.robot_constraints_dict["max_linear_velocity"],
                                            node.robot_constraints_dict["max_linear_velocity"])
                node.pub_cmd.publish(cmd_msg)
                node.last_v = next_v
                # node.last_w = next_w

            else:
                rospy.loginfo("goal reached! {:.2f}".format(dis_robot2goal))
                # Stop the robot
                cmd_msg = Twist()
                accel_linear = proportional_control(0, node.robot_twist.linear.x, SPEED_PROPORTIONAL_GAIN)
                cmd_msg.linear.x = np.clip(node.robot_twist.linear.x + accel_linear * dt,
                                            0.0,
                                            node.robot_constraints_dict["max_linear_velocity"])
                node.pub_cmd.publish(cmd_msg)
                node.pub_tracking_progress.publish(1.0)

        rate.sleep()


#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

flag_busy = False
linear_velocity = 0
angular_velocity = 0

def cmd_cb(msg):
    global linear_velocity
    global angular_velocity

    flag_busy = True
    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z
    flag_busy = False


if __name__ == '__main__':
    rospy.init_node('fake_odom_node', anonymous=False)

    pub_odom = rospy.Publisher("odom", Odometry, queue_size=1)
    sub_cmd = rospy.Subscriber("cmd_vel", Twist, cmd_cb, queue_size=1)
    odom_broadcaster = tf.TransformBroadcaster()
    odom_msg = Odometry()
    odom_msg.header.frame_id = "odom"
    odom_msg.child_frame_id = "base_link"

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.1
    vy = -0.1
    vth = 0.1

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # Block current process while the velocity command is updating
        while flag_busy:
            pass

        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - last_time).to_sec()
        x += linear_velocity * cos(th) * dt;
        y += linear_velocity * sin(th) * dt;
        th += angular_velocity * dt;

        # rospy.logwarn("v,w: {:.2f},{:.2f}\tpose:({:.2f}, {:.2f}, {:.2f})".format(linear_velocity, angular_velocity, x, y, th))

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # first, we'll publish the transform over tf
        odom_broadcaster.sendTransform(
            (x, y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom_msg.header.stamp = current_time
        odom_msg.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
        odom_msg.twist.twist = Twist(Vector3(linear_velocity, 0, 0), Vector3(0, 0, angular_velocity))

        # publish the message
        pub_odom.publish(odom_msg)

        last_time = current_time
        r.sleep()
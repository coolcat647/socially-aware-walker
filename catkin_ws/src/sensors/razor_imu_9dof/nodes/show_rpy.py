#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Quaternion
import math

pub_pose = None

def callback(msg):
    q = (
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    )
    (roll, pitch, yaw) = euler_from_quaternion(q)
    # print('yaw= {}, pitch={}, roll={}'.format(yaw, pitch, roll))
    # print('yaw= {}, pitch={}, roll={}'.format(yaw * 180.0 / math.pi, pitch * 180.0 / math.pi, roll * 180.0 / math.pi))

    global pub_pose
    if pub_pose is not None:
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "odom" # msg.header.frame_id
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.orientation = msg.orientation
        pub_pose.publish(pose_msg)

if __name__ == '__main__':
    rospy.init_node('show_rpy_node', anonymous=True)
    rospy.Subscriber("/walker/imu", Imu, callback)

    # global pub_pose
    pub_pose = rospy.Publisher("imu_vis", PoseStamped, queue_size=1)

    rospy.spin()

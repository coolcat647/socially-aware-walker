#!/usr/bin/env python3
import numpy as np
from datetime import datetime
import os
from tqdm import tqdm
from pymouse import PyMouse # For auto rviz reset

import rospy
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose2D, PoseStamped, Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest
from nav_msgs.msg import Odometry

NUM_AGENTS = 8
# NUM_CASES = 200
TIME_LIMIT_IN_SEC = 35.0 + 1.0


class _WFM(object):
    def __init__(self):
        self.msg = None
    def cb(self, msg):
        if self.msg is None:
            self.msg = msg
            
def my_wait_for_message(topic, topic_type, timeout=None):
    wfm = _WFM()
    s = None
    try:
        s = rospy.topics.Subscriber(topic, topic_type, wfm.cb)
        if timeout is not None:
            # timeout_t = time.time() + timeout
            if isinstance(timeout, rospy.Duration):
                timeout_t = rospy.Time.now() + timeout
            else:
                timeout_t = rospy.Time.now() + rospy.Duration(timeout)

            while not rospy.core.is_shutdown() and wfm.msg is None:
                rospy.sleep(0.01)
                if rospy.Time.now() >= timeout_t:
                    raise rospy.exceptions.ROSException("timeout exceeded while waiting for message on topic %s"%topic)

        else:
            while not rospy.core.is_shutdown() and wfm.msg is None:
                rospy.sleep(0.01)            
    finally:
        if s is not None:
            s.unregister()
    if rospy.core.is_shutdown():
        raise rospy.exceptions.ROSInterruptException("rospy shutdown")
    return wfm.msg



if __name__ == '__main__':
    rospy.init_node("gym_auto_runner", anonymous=False)

    # For auto rviz reset
    sim_mouse = PyMouse()

    # ROS Publisher
    pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    # ROS Service Client
    rospy.wait_for_service("/gazebo/reset_world", timeout=1)
    gz_reset_pose = rospy.ServiceProxy("/gazebo/reset_world", Empty)
    # rospy.wait_for_service("/pedsim_simulator/pause_simulation", timeout=1)
    # pause_simulation = rospy.ServiceProxy("/pedsim_simulator/pause_simulation", Empty)
    # pause_simulation(EmptyRequest())
    rospy.wait_for_service("walker/cancel_navigation", timeout=1)
    cancel_navigation = rospy.ServiceProxy("walker/cancel_navigation", Empty)
    cancel_navigation(EmptyRequest())

    progress_bar = tqdm(range(0, 1))
    for idx in progress_bar:
        progress_bar.set_description("Process gym case {}".format(idx + 1))
        gz_reset_pose(EmptyRequest())
        rospy.sleep(1.0)
        sim_mouse.click(int(sim_mouse.screen_size()[0] / 2 + 30),
                        int(sim_mouse.screen_size()[1] - 10),
                        1)

        # cmd_str = "rosrun pedsim_simulator gym_reset_node.py --num_agents {} --use_testcase --testcase {}".format(NUM_AGENTS, idx + 1)
        # os.system(cmd_str)
        # rospy.sleep(1.0)
        
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "odom"
        goal_msg.pose.position.y = 4.0
        goal_msg.pose.orientation.w = 1.0
        goal_msg.header.stamp = rospy.Time.now()
        pub_goal.publish(goal_msg)

        flag_arrival = True
        failure_message = ""
        try:
            flag_arrival_msg = my_wait_for_message("/walker/flag_arrival", Bool, timeout=TIME_LIMIT_IN_SEC)

            if flag_arrival_msg.data == False: 
                flag_arrival = False
                failure_message = "collision"
        except rospy.ROSException as e:
            flag_arrival = False
            failure_message = "timeout"

        cancel_navigation(EmptyRequest())
        if flag_arrival == False:
            testcase = rospy.get_param("/walker/testcase") if rospy.has_param("/walker/testcase") else 0
            rospy.logwarn("Task failed: {} -- {}".format(testcase, failure_message))
        rospy.sleep(1.5)
        
        

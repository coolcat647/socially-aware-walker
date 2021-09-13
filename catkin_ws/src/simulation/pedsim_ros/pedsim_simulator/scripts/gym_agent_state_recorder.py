#!/usr/bin/env python3
import numpy as np
from datetime import datetime
import re

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, PoseStamped, Point
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty, EmptyRequest

ROBOT_GAZEBO_MODEL_NAME = "walker"
TIME_LIMIT_IN_SEC = 30.0


class GymAgentStateRecorder(object):
    def __init__(self):

        # ROS subscriber
        self.sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        self.sub_progress = rospy.Subscriber("/walker/tracking_progress", Float32, self.progress_cb, queue_size=1)

        self.flag_recording = False
        self.flag_arrival = False
        self.csvfile = None
        self.start_navi_time = None
        self.cnt_missing_msg = 0

        try:
            # Get number of agents from the gazebo msg
            states_msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=0.5)
            actors_list = [actor_name for actor_name in states_msg.name if "actor" in actor_name]
            self.num_agents = len(actors_list)

            # Check if robot model exists in gazebo model list 
            if not ROBOT_GAZEBO_MODEL_NAME in states_msg.name:
                rospy.logerr("Miss robot model in gazebo model_states msg, aborting...")
                exit(-1)
            
        except rospy.ROSException as e:
            self.num_agents = 0
            rospy.logwarn("Cannot get any model_states msg, aborting...")
            exit(-1)

        # Signal callback setup 
        rospy.on_shutdown(self.shutdown_cb)

        # ROS timer
        self.timer = rospy.Timer(rospy.Duration(0.25), self.timer_cb)
        rospy.loginfo(rospy.get_name() + " is ready.")


    def check_and_close_csvfile(self):
        if self.csvfile is not None:
            self.csvfile.close()


    def timer_cb(self, event):
        # If not recording, just skip
        if not self.flag_recording: return
        
        states_msg = None
        try:
            states_msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=0.2)
            # Reset message missing count
            self.cnt_missing_msg = 0
        except rospy.ROSException as e:
            self.cnt_missing_msg += 1
            if self.cnt_missing_msg >= 10:
                rospy.logerr("Miss model_states msg too many times, aborting...")
                self.check_and_close_csvfile()
                exit(-1)

        # Collect all agents position
        robot_position = Point()
        agent_position_list = [Point(),] * self.num_agents
        for idx, model_name in enumerate(states_msg.name):
            if model_name == ROBOT_GAZEBO_MODEL_NAME:
                robot_position = states_msg.pose[idx].position
            else:
                found_list = re.findall(r"actor_\d+", model_name)
                if len(found_list) != 0:
                    actor_num = int(found_list[0].split("_")[1])
                    agent_position_list[actor_num - 1] = states_msg.pose[idx].position

        # Calculate the min distance between the robot and agents
        distance_list = [np.hypot(robot_position.x - tmp.x, robot_position.y - tmp.y) for tmp in agent_position_list]

        # Put the time stamp, agent position, min distance, and arrival check result into the string
        elapsed_time = (rospy.Time.now() - self.start_navi_time).to_sec()
        row_string = "{:.3f},{:.2f},{:.2f},".format(elapsed_time, robot_position.x, robot_position.y)
        for agent_position in agent_position_list:
            row_string += "{:.2f},{:.2f},".format(agent_position.x, agent_position.y)
        row_string += "{:.2f},".format(min(distance_list))

        if self.flag_arrival:
            row_string += "yes\n"
            self.csvfile.write(row_string)
            rospy.loginfo("Goal arrival, stop recording")
            self.flag_recording = False
            self.flag_arrival = False
            self.csvfile.close()
        else:
            row_string += "no\n"
            self.csvfile.write(row_string)
        # print(row_string)

        # Timeout handling
        if elapsed_time >= TIME_LIMIT_IN_SEC:
            rospy.logwarn("Run out of the time, stop recording")
            self.flag_recording = False
            self.flag_arrival = False
            self.csvfile.close()


    def goal_cb(self, msg):
        self.flag_recording = True
        if rospy.has_param('testcase'):
            testcase = rospy.get_param('testcase')
        else:
            testcase = 0

        # Close old csvfile
        self.check_and_close_csvfile()

        now = datetime.now()
        date_time = now.strftime("%Y%m%d%H%M%S")
        # testcase          : 1-500, 0->random
        # humans number     : 8
        # moving distance   : 8 m
        # agendt speed      : 1.34-0.26 m/s
        rospy.loginfo("Start recording: gym_case{}_h{}_d8_t{}.csv".format(testcase, self.num_agents, date_time))
        self.csvfile = open("gym_case{}_h{}_d8_t{}.csv".format(testcase, self.num_agents, date_time), "w")
        header_string = "time stamp,pxr,pyr,"
        for idx in range(self.num_agents):
            header_string += "px{},py{},".format(idx + 1, idx + 1)
        header_string += "min distance,succ\n"
        self.csvfile.write(header_string)

        self.flag_arrival = False
        self.start_navi_time = rospy.Time.now()


    def progress_cb(self, msg):
        if msg.data == 1.0:
            self.flag_arrival = True    # Goal arrival

    def shutdown_cb(self):
        # Close csvfile
        self.check_and_close_csvfile()
        rospy.loginfo("Shutdown " + rospy.get_name())

    
        


if __name__ == '__main__':
    rospy.init_node("gym_agent_state_recorder", anonymous=False)

    # Service Client
    # rospy.wait_for_service("/pedsim_simulator/unpause_simulation", timeout=3)
    # pedsim_unpause = rospy.ServiceProxy("/pedsim_simulator/unpause_simulation", Empty)

    state_recorder = GymAgentStateRecorder()
    rospy.spin()
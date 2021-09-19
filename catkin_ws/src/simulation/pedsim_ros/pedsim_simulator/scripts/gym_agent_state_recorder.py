#!/usr/bin/env python3
import numpy as np
from datetime import datetime
import re
import os

import rospy
import rospkg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose2D, PoseStamped, Point
from gazebo_msgs.msg import ModelStates, ContactsState
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest

ROBOT_GAZEBO_MODEL_NAME = "walker"
TIME_LIMIT_IN_SEC = 35.0


class GymAgentStateRecorder(object):
    def __init__(self):

        # ROS subscriber
        self.sub_goal = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=1)
        self.sub_bumper = rospy.Subscriber("bumper", ContactsState, self.bumper_cb, queue_size=1)
        self.pub_arrival = rospy.Publisher("flag_arrival", Bool, queue_size=1)

        self.csvfile = None
        self.flag_recording = False
        self.flag_arrival = False
        self.start_navi_time = None
        self.cnt_missing_msg = 0
        self.goal_position = Point()
        self.model_collided = None

        try:
            # Get number of agents from the gazebo msg
            states_msg = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=0.5)
            actors_list = [actor_name for actor_name in states_msg.name if "actor" in actor_name]
            self.num_agents = len(actors_list)

            # Check if robot model exists in gazebo model list 
            if not ROBOT_GAZEBO_MODEL_NAME in states_msg.name:
                rospy.logerr("Missing robot model in gazebo model_states msg, aborting...")
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
        if self.csvfile is not None and not self.csvfile.closed:
            self.csvfile.close()


    def check_and_write_csvfile(self, row_str):
        if self.csvfile is not None and not self.csvfile.closed:
            self.csvfile.write(row_str)
        else:
            rospy.logwarn("Conflicts in writing file.")


    def bumper_cb(self, msg):
        if len(msg.states) > 0:
            for bumper_state in msg.states:
                self.model_collided = bumper_state.collision2_name.split("::")[0]
                break



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
        robot_pose = Pose2D()
        agent_pose_list = [Pose2D() for i in range(self.num_agents)]
        for idx, model_name in enumerate(states_msg.name):
            if model_name == ROBOT_GAZEBO_MODEL_NAME:
                robot_pose.x = states_msg.pose[idx].position.x
                robot_pose.y = states_msg.pose[idx].position.y
                robot_pose.theta = euler_from_quaternion([states_msg.pose[idx].orientation.x,
                                                          states_msg.pose[idx].orientation.y,
                                                          states_msg.pose[idx].orientation.z,
                                                          states_msg.pose[idx].orientation.w])[2]
            else:
                found_list = re.findall(r"actor_\d+", model_name)
                if len(found_list) != 0:
                    actor_num = int(found_list[0].split("_")[1])
                    agent_pose_list[actor_num - 1].x = states_msg.pose[idx].position.x
                    agent_pose_list[actor_num - 1].y = states_msg.pose[idx].position.y
                    agent_pose_list[actor_num - 1].theta = euler_from_quaternion([
                                                                states_msg.pose[idx].orientation.x,
                                                                states_msg.pose[idx].orientation.y,
                                                                states_msg.pose[idx].orientation.z,
                                                                states_msg.pose[idx].orientation.w])[2]

        # Calculate the min distance between the robot and agents
        distance_list = [np.hypot(robot_pose.x - tmp.x, robot_pose.y - tmp.y) for tmp in agent_pose_list]

        # Check if goal arrival
        if np.hypot(robot_pose.x - self.goal_position.x, robot_pose.y - self.goal_position.y) < 0.4:
            self.flag_arrival = True

        # Put the time stamp, agent position, min distance, and arrival check result into the string
        elapsed_time = (rospy.Time.now() - self.start_navi_time).to_sec()
        row_string = "{:.3f},{:.2f},{:.2f},{:.2f},".format(elapsed_time,
                                                           robot_pose.x,
                                                           robot_pose.y,
                                                           robot_pose.theta)
        for agent_position in agent_pose_list:
            row_string += "{:.2f},{:.2f},{:.2f},".format(agent_position.x,
                                                         agent_position.y,
                                                         agent_position.theta)
        row_string += "{:.2f},".format(min(distance_list))


        if self.model_collided is not None:
            row_string += "{},".format(self.model_collided)     # collision flag
            row_string += "no\n"                                # success flag
            self.check_and_write_csvfile(row_string)
            self.check_and_close_csvfile()
            rospy.logwarn("Collision occurred, stop recording")
            self.flag_recording = False
            self.flag_arrival = False
            self.model_collided = None                          # collision state init
            self.pub_arrival.publish(False)

        elif self.flag_arrival:
            row_string += "no,"     # collision flag
            row_string += "yes\n"   # success flag
            self.check_and_write_csvfile(row_string)
            self.check_and_close_csvfile()
            rospy.loginfo("Goal arrival, stop recording")
            self.flag_recording = False
            self.flag_arrival = False
            self.pub_arrival.publish(True)
        else:
            row_string += "no,"     # collision flag
            row_string += "no\n"    # success flag
            self.check_and_write_csvfile(row_string)
        # print(row_string)

        # Timeout handling
        if elapsed_time >= TIME_LIMIT_IN_SEC:
            rospy.logwarn("Run out of the time, stop recording")
            self.flag_recording = False
            self.flag_arrival = False
            self.check_and_close_csvfile()
            # self.pub_arrival.publish(False)


    def goal_cb(self, msg):
        # Set goal position
        self.goal_position.x = msg.pose.position.x
        self.goal_position.y = msg.pose.position.y

        # Get testcase
        testcase = rospy.get_param("testcase") if rospy.has_param("testcase") else 0
        navi_approach = rospy.get_param("navi_approach") if rospy.get_param("navi_approach") else "UnkownMethod"

        # Close old csvfile
        self.check_and_close_csvfile()

        rospack = rospkg.RosPack()
        target_directory = os.path.abspath(os.path.join(rospack.get_path("pedsim_simulator"), "../../../.."))

        now = datetime.now()
        date_time = now.strftime("%Y%m%d%H%M%S")
        # testcase          : 1-500, 0->random
        # humans number     : 8
        # moving distance   : 8 m
        # agendt speed      : 1.34-0.26 m/s
        filename = "{}_case{}_h{}_d8_v05_t{}.csv".format(navi_approach, testcase, self.num_agents, date_time)
        rospy.loginfo("Start recording: {}".format(filename))
        self.csvfile = open(os.path.join(target_directory, filename), "w")
        header_string = "time stamp,pxr,pyr,thetar,"
        for idx in range(self.num_agents):
            header_string += "px{},py{},theta{},".format(idx + 1, idx + 1, idx + 1)
        header_string += "min distance,collision,succ\n"
        self.csvfile.write(header_string)

        self.flag_arrival = False
        self.start_navi_time = rospy.Time.now()
        self.flag_recording = True
        self.model_collided = None


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
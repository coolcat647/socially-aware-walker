#!/usr/bin/env python
"""
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
from rospkg import RosPack
from pedsim_msgs.msg  import AgentStates

# xml file containing a gazebo model to represent agent, currently is represented by a cubic but can be changed
global xml_file

# Samliu
from std_srvs.srv import Empty
AGENT_TYPE_LIST = ['adult', 'child', 'robot', 'elder']


def actor_poses_callback(actors):
    for actor in actors.agent_states:
        actor_id = str( actor.id )
        actor_pose = actor.pose

        rospy.loginfo("Spawning actor id:{}, location:({:.2f}, {:.2f})".format(actor_id, actor_pose.position.x, actor_pose.position.y))
        model_pose = Pose(Point(x= actor_pose.position.x,
                               y= actor_pose.position.y,
                               z= actor_pose.position.z),
                         Quaternion(actor_pose.orientation.x,
                                    actor_pose.orientation.y,
                                    actor_pose.orientation.z,
                                    actor_pose.orientation.w) )

        # spawn_model(actor_id, xml_string, "", model_pose, "world")
        # Samliu
        spawn_model("actor_{}_{}".format(actor_id, AGENT_TYPE_LIST[actor.type]), xml_string, "", model_pose, "world")

    ##### Method 1 for pause the simulation #####
    # import dynamic_reconfigure.client
    # client = dynamic_reconfigure.client.Client("pedsim_simulator", timeout=30)
    # client.update_configuration({"paused": True, })

    ##### Method 2 for pause the simulation #####
    rospy.wait_for_service("/pedsim_simulator/unpause_simulation", timeout=30)
    pause_simulation = rospy.ServiceProxy("/pedsim_simulator/pause_simulation", Empty)
    pause_simulation()

    rospy.signal_shutdown("all agents have been spawned !")


if __name__ == '__main__':

    rospy.init_node("spawn_pedsim_agents")

    rospack1 = RosPack()
    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
    default_actor_model_file = pkg_path + "/models/actor_model.sdf"

    actor_model_file = rospy.get_param('~actor_model_file', default_actor_model_file)
    file_xml = open(actor_model_file)
    xml_string = file_xml.read()

    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    print("service: spawn_sdf_model is available ....")
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, actor_poses_callback)

    rospy.spin()

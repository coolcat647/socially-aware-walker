#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import math
import time
import copy
import numpy as np
from filterpy.kalman import KalmanFilter

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import WrenchStamped, Vector3



class ForceFilteringNode(object):
    def __init__(self):
        # Kalman filter
        if rospy.get_param('~use_only_main_force', True):
            self.force_data_dimension = 2       # force y, torque z
        else:
            self.force_data_dimension = 6       # force xyz, torque xyz

        self.kf = KalmanFilter(dim_x=self.force_data_dimension, dim_z=self.force_data_dimension)       
        self.kf.F = np.eye(self.force_data_dimension)      # state transition matrix

        self.kf.H = np.eye(self.force_data_dimension)      # measurement function,

        

        self.kf.R[0:,0:] *= 10.0   # measurement uncertainty         
        self.kf.P *= 0.01

        self.kf.Q[:, :] *= 2.0  # process uncertainty
        
        if rospy.get_param('~use_default_offset', False):
            self.force_offset = np.array([-12.50041, -14.795424, -14.968935, 2.86582, 0.23863193, 0.0706105])
        else:
            # Calculate the offset by force averaging
            force_data_array = None
            for i in range(1, 20*4):
                try:
                    msg = rospy.wait_for_message('force', WrenchStamped, timeout=0.5)
                except rospy.ROSException as e:
                    rospy.logerr("Timeout while waiting for force data!")
                    exit(-1)

                if self.force_data_dimension == 2:
                    tmp_force = np.array([msg.wrench.force.y, msg.wrench.torque.z], dtype=np.float32)
                else:
                    tmp_force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                                        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z], dtype=np.float32)

                if force_data_array is None:
                    force_data_array = tmp_force
                else:
                    force_data_array = np.vstack((force_data_array, tmp_force))

            # Force offset & initial value
            self.force_offset = np.average(force_data_array, axis=0)
            self.kf.x[:] = (force_data_array[-1, :] - self.force_offset).reshape((self.force_data_dimension, 1))
        print("default force offset:", self.force_offset)

        self.pub_force = rospy.Publisher('force_filtered', WrenchStamped, queue_size=1)
        self.sub_force = rospy.Subscriber("force", WrenchStamped, self.force_cb, queue_size=1)
        print(rospy.get_name() + ' is ready.')
        

    def force_cb(self, msg):
        # start_crawling_time = time.time()

        if self.force_data_dimension == 6:
            force_raw = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
                                msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z], dtype=np.float32)
            force_tmp = force_raw - self.force_offset

            # Kalman filter predict
            self.kf.predict()
            # Kalman filter update
            self.kf.update(force_tmp)
            force_filtered = self.kf.x.reshape((self.force_data_dimension, ))

            output_msg = WrenchStamped()
            output_msg.header.frame_id = msg.header.frame_id
            output_msg.wrench.force = Vector3(x=force_filtered[0], y=force_filtered[1], z=force_filtered[2])
            output_msg.wrench.torque = Vector3(x=force_filtered[3], y=force_filtered[4], z=force_filtered[5])
            output_msg.header.stamp = msg.header.stamp #rospy.Time.now()
            self.pub_force.publish(output_msg)

        elif self.force_data_dimension == 2:
            self.force_offset_main = self.force_offset[[1, -1]]
            force_raw = np.array([msg.wrench.force.y, msg.wrench.torque.z], dtype=np.float32)
            force_tmp = force_raw - self.force_offset_main

            # force_tmp = np.array([force_tmp[1], force_tmp[5]], dtype=np.float32)

            # Kalman filter predict
            self.kf.predict()
            # Kalman filter update
            self.kf.update(force_tmp)
            force_filtered = self.kf.x.reshape((self.force_data_dimension, ))

            output_msg = WrenchStamped()
            output_msg.header.frame_id = msg.header.frame_id
            output_msg.wrench.force = Vector3(x=0, y=force_filtered[0], z=0)
            output_msg.wrench.torque = Vector3(x=0, y=0, z=force_filtered[1])
            output_msg.header.stamp = msg.header.stamp #rospy.Time.now()
            self.pub_force.publish(output_msg)

        else:
            rospy.logerr("Undefined filter data dimension") 

        # end_crawling_time = time.time()
        # print("Total execution time: {:.5f} seconds".format(end_crawling_time - start_crawling_time))
        

if __name__ == '__main__':
    rospy.init_node('force_extraction_node', anonymous=False)
    node = ForceFilteringNode()
    rospy.spin()

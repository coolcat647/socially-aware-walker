#!/usr/bin/env python3

# -*- coding: utf-8 -*-
'''
@Time          : 20/04/25 15:49
@Author        : huguanghao
@File          : demo.py
@Noice         :
@Modificattion :
    @Author    :
    @Time      :
    @Detail    :

'''

import argparse
import os
import sys
import math
import time
import copy
import numpy as np
from AB3DMOT_libs.model import AB3DMOT

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from walker_msgs.msg import Det3D, Det3DArray


INTEREST_CLASSES = ["person"]
MARKER_LIFETIME = 0.2


def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


class MultiObjectTrackingNode(object):
    def __init__(self):
        rospy.on_shutdown(self.shutdown_cb)

        # Tracker
        self.mot_tracker = AB3DMOT(max_age=7)

        self.pub_trk = rospy.Publisher('tracking_result', MarkerArray, queue_size=1)
        self.sub_image = rospy.Subscriber("det3d_result", Det3DArray, self.det_result_cb, queue_size=1)
        
        print(rospy.get_name() + ' is ready.')


    def det_result_cb(self, msg):
        if len(msg.dets_list) == 0:
            return
        dets_list = None
        info_list = None
        for idx, det in enumerate(msg.dets_list):
            if dets_list is None:
                dets_list = np.array([det.h, det.w, det.l, det.x, det.y, det.z, det.yaw], dtype=np.float32)
                info_list = np.array([det.confidence, ], dtype=np.float32)
            else:
                dets_list = np.vstack([dets_list, [det.h, det.w, det.l, det.x, det.y, det.z, det.yaw]])
                info_list = np.vstack([info_list, [det.confidence, ]])

        # if no detection in a sequence
        if len(dets_list.shape) == 1: dets_list = np.expand_dims(dets_list, axis=0)
        if len(info_list.shape) == 1: info_list = np.expand_dims(info_list, axis=0)
        if dets_list.shape[1] == 0:
            return

        dets_all = {'dets': dets_list, 'info': info_list}
        # important
        start_time = time.time()
        trackers = self.mot_tracker.update(dets_all)
        cycle_time = time.time() - start_time

        # saving results, loop over each tracklet           
        marker_array = MarkerArray()
        for idx, d in enumerate(trackers):
            vx = d[7]
            vy = d[8]
            vz = d[9]
            speed = np.sqrt(vx**2 + vy**2) * 10     # Note: reconstruct speed by multipling the sampling rate 
            # print("speed: {}".format(speed))
            yaw = np.arctan2(d[8], d[7]) 

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.ns = 'object'
            marker.id = idx
            marker.lifetime = rospy.Duration(MARKER_LIFETIME)#The lifetime of the bounding-box, you can modify it according to the power of your machine.
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.scale.x = d[2]
            marker.scale.y = d[1]
            marker.scale.z = d[0]
            marker.color.b = 1.0
            marker.color.a = 0.25 #The alpha of the bounding-box
            marker.pose.position.x = d[3]
            marker.pose.position.y = d[4]
            marker.pose.position.z = d[5]
            q = euler_to_quaternion(0, 0, yaw)
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
            marker_array.markers.append(marker)

            str_marker = Marker()
            str_marker.header.frame_id = msg.header.frame_id
            str_marker.header.stamp = rospy.Time.now()
            str_marker.ns = 'text'
            str_marker.id = idx
            str_marker.scale.z = 0.4 #The size of the text
            str_marker.color.b = 1.0
            str_marker.color.g = 1.0
            str_marker.color.r = 1.0
            str_marker.color.a = 1.0
            str_marker.pose.position.x = d[3]
            str_marker.pose.position.y = d[4]
            str_marker.pose.position.z = d[5]
            str_marker.lifetime = rospy.Duration(MARKER_LIFETIME)
            str_marker.type = Marker.TEXT_VIEW_FACING
            str_marker.action = Marker.ADD
            str_marker.text = str(d[10]) # + ' {:.3f}'.format(result['score']) #for visualize detection
            marker_array.markers.append(str_marker)

            
            arrow_marker = copy.deepcopy(marker)
            arrow_marker.type = Marker.ARROW
            arrow_marker.ns = 'direction'
            arrow_marker.scale.x = 1.0 * (speed / 1.5)
            arrow_marker.scale.y = 0.2
            arrow_marker.scale.z = 0.2
            marker_array.markers.append(arrow_marker)


            # bbox3d_tmp = d[0:7]       # h, w, l, x, y, z, theta in camera coordinate
            # id_tmp = d[7]
            # ori_tmp = d[8]
            # type_tmp = det_id2str[d[9]]
            # bbox2d_tmp_trk = d[10:14]
            # conf_tmp = d[14]
        self.pub_trk.publish(marker_array)

        # print("elapsed time: {}".format(cycle_time))


        


    def shutdown_cb(self):
        rospy.loginfo("Shutdown " + rospy.get_name())


if __name__ == '__main__':
    rospy.init_node('mot_node', anonymous=False)

    node = MultiObjectTrackingNode()
    rospy.spin()

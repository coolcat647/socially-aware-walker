#!/usr/bin/env bash
rosbag record --duration 60 -o new_bag \
            /walker/scan \
            /walker/usb_cam/image_raw \
            /walker/usb_cam/camera_info \
            /tf \
            /tf_static \
            /walker/force_filtered \
            /walker/odom_filtered \
            /walker/wheel_odom \
            /walker/imu \
            /walker/cmd_vel \
            /walker/inhibition_force \
            /walker/system_torque \
            /walker/inhibiton \
            /walker/walkable_path \
            /walker/smooth_path \
            /walker/trk3d_result \

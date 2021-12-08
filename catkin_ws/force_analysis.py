#!/usr/bin/env python3
import csv
import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import matplotlib.lines as mlines
from matplotlib import patches
from decimal import Decimal


def plot_force3(data_dict):
    # fig = plt.figure(figsize=(12, 9))
    fig, ax_list = plt.subplots(4, 1, figsize=(12, 8), gridspec_kw={'height_ratios': [2.5, 1.5, 1.5, 1.5]})
    # fig, ax = plt.subplots(1, 1, figsize=(12, 3))
    
    user_force = np.vstack((data_dict["/walker/force_filtered/wrench/force/y_x"],
                                 data_dict["/walker/force_filtered/wrench/force/y_y"]))
    inhibition_force = np.vstack((data_dict["/walker/inhibition_force/data_x"],
                                  data_dict["/walker/inhibition_force/data_y"]))
    system_torque = np.vstack((data_dict["/walker/system_torque/data_x"],
                               data_dict["/walker/system_torque/data_y"]))
    linear_velocity = np.vstack((data_dict["/walker/cmd_vel/linear/x_x"],
                                 data_dict["/walker/cmd_vel/linear/x_y"]))
    angular_velocity = np.vstack((data_dict["/walker/odom_filtered/twist/twist/angular/z_x"],
                                  data_dict["/walker/odom_filtered/twist/twist/angular/z_y"]))


    print(data_dict["/walker/force_filtered/wrench/force/y_x"])



    # Scenario A
    time_start = 1636133751 + 7.8

    user_force[0] -= time_start
    inhibition_force[0] -= time_start
    system_torque[0] -= time_start
    linear_velocity[0] -= time_start
    angular_velocity[0] -= time_start
    
    user_force[1] = np.clip(user_force[1], 0, 999)

    user_force_line, = ax_list[0].plot(user_force[0], user_force[1])
    inhibition_force_line, = ax_list[0].plot(inhibition_force[0], inhibition_force[1], color="red")
    # ax_list[0].legend([user_force_line, inhibition_force_line],
    #                   ["Simulated user pushing force", "Force provided by the robot"],
    #                   loc='upper right', fontsize=12)
    ax_list[0].legend([user_force_line, inhibition_force_line],
                      ["User-applied force", "Force provided by the robot"],
                      loc='upper right', fontsize=12)

    # plt.xlabel("t (seconds)", fontsize=12)
    ax_list[0].set_ylabel("Force: (N)", fontsize=18)
    ax_list[0].set_yticks(np.linspace(0, 60, 7))
    # ax_list[0].set_xticks(np.linspace(0, 35, 8))
    

    # Scenario A
    ax_list[0].set_xticks(list(ax_list[0].get_xticks()) + [8.0, 14.5, 26.1, 33.8, 36.4, 47.3] + [58])
    # ax_list[0].text(x=3.0, y=35, s="Inhibition force", color="red", fontsize=12)
    ax_list[0].text(x=33, y=47.5, s=r"$f_{inh}$", color="red", fontsize=16)
    ax_list[0].arrow(34, 45, -6.5, -25, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    ax_list[0].arrow(34, 45, -0, -25, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    ax_list[0].arrow(34, 45, 1.3, -17, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    ax_list[0].arrow(34, 45, 12.3, -25, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)

    # ax_list[0].text(x=15.0, y=35, s="Estop force", color="gray", fontsize=12)
    ax_list[0].text(x=13.5, y=47, s=r"$f_{estop}$", color="red", fontsize=16)
    ax_list[0].text(x=56.5, y=35, s=r"$f_{estop}$", color="red", fontsize=12)
    # ax_list[0].arrow(16, 35, -3.5, -8, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax_list[0].arrow(16, 35, -3, -12, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax_list[0].arrow(16, 35, -2, -12, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax_list[0].arrow(16, 35, 8.0, -15, head_width=0.8, head_length=1.2, width=0.001, ec ='gray', alpha=0.5)

    import matplotlib.patches as patches

    arrow = patches.FancyArrowPatch((7.8, 40), (10.2, 40), arrowstyle='<->', mutation_scale=10)
    ax_list[0].text(x=7.8, y=45, s=r"$T_{gait}$", color="gray", fontsize=14)
    ax_list[0].add_artist(arrow)

    # Scenario B
    # ax_list[0].set_xticks(list(ax_list[0].get_xticks()) + [16.32] + [])
    # # ax_list[0].text(x=14.0, y=35, s="Inhibition force", color="red", fontsize=12)
    # ax_list[0].text(x=15.5, y=37, s=r"$f_{inh}$", color="red", fontsize=16)
    # ax_list[0].arrow(16, 35, 0.2, -17, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # # ax_list[0].arrow(5, 35, -1, -12, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # # ax_list[0].arrow(5, 35, 1.3, -17, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)

    # # ax_list[0].text(x=10.0, y=35, s="Estop force", color="gray", fontsize=12)
    # ax_list[0].text(x=11.50, y=37, s=r"$f_{estop}$", color="red", fontsize=16)
    # ax_list[0].arrow(12, 35, -6.2, -12, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax_list[0].arrow(12, 35, -0.7, -5, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax_list[0].arrow(12, 35, 8.8, -10, head_width=0.8, head_length=1.2, width=0.001, ec ='gray', alpha=0.5)

    ax_list[0].set_ylim([-2.5, 60])
    ax_list[0].set_xlim([-2.5, 60])
    ax_list[0].grid(True, alpha=0.5)
    # ax_list[0].set_xlabel("t (seconds)", fontsize=16)


    #### Plot linear velocity ####
    linear_velocity_line, = ax_list[1].plot(linear_velocity[0], linear_velocity[1])
    plt.xlabel("t (seconds)", fontsize=12)
    ax_list[1].set_ylabel(r"$v$" + " : (m/s)", fontsize=14)
    ax_list[1].set_yticks(np.linspace(-0.25, 1.0, 6))
    # ax_list[1].set_xticks(np.linspace(0, 35, 8))
    ax_list[1].set_xlim([-2.5, 60])
    ax_list[1].grid(True, alpha=0.5)

    #### Plot torque ####
    system_torque_line, = ax_list[2].plot(system_torque[0], system_torque[1])
    plt.xlabel("t (seconds)", fontsize=12)
    ax_list[2].set_ylabel("Driving torque:\n(N·m)", fontsize=14)
    ax_list[2].set_yticks(np.linspace(-10, 40, 6))
    # ax_list[2].set_xticks(np.linspace(0, 35, 8))
    ax_list[2].set_xlim([-2.5, 60])
    ax_list[2].grid(True, alpha=0.5)

    #### Plot angular velocity ####
    angular_velocity_line, = ax_list[3].plot(angular_velocity[0], angular_velocity[1])
    ax_list[3].set_xlabel("t (seconds)", fontsize=12)
    ax_list[3].set_ylabel(r"$\omega$" + " : (1/s)", fontsize=14)
    ax_list[3].set_yticks(np.linspace(-0.75, 0.75, 7))
    # ax_list[3].set_xticks(np.linspace(0, 35, 8))
    ax_list[3].set_xlim([-2.5, 60])
    ax_list[3].grid(True, alpha=0.5)

    plt.subplots_adjust(top=0.98, bottom=0.08, left=0.08, right=0.97, hspace=0.3)
    plt.show()

'''
def plot_force2(data_dict):
    # fig = plt.figure(figsize=(12, 9))
    # fig, ax_list = plt.subplots(4, 1, figsize=(12, 8), gridspec_kw={'height_ratios': [2.5, 1.5, 1.5, 1.5]})
    fig, ax = plt.subplots(1, 1, figsize=(12, 3))
    
    user_force = np.vstack((data_dict["/walker/force_filtered/wrench/force/y_x"],
                                 data_dict["/walker/force_filtered/wrench/force/y_y"]))
    inhibition_force = np.vstack((data_dict["/walker/inhibition_force/data_x"],
                                  data_dict["/walker/inhibition_force/data_y"]))
    system_torque = np.vstack((data_dict["/walker/system_torque/data_x"],
                               data_dict["/walker/system_torque/data_y"]))
    linear_velocity = np.vstack((data_dict["/walker/odom_filtered/twist/twist/linear/x_x"],
                                 data_dict["/walker/odom_filtered/twist/twist/linear/x_y"]))
    angular_velocity = np.vstack((data_dict["/walker/odom_filtered/twist/twist/angular/z_x"],
                                  data_dict["/walker/odom_filtered/twist/twist/angular/z_y"]))
    # Scenario A
    time_start = 4.29
    
    # Scenario B
    # time_start = 2.84


    user_force[0] -= time_start
    inhibition_force[0] -= time_start
    system_torque[0] -= time_start
    linear_velocity[0] -= time_start
    angular_velocity[0] -= time_start

    # ax = fig.add_subplot(411)
    user_force_line, = ax.plot(user_force[0], user_force[1])
    # inhibition_force_line, = ax.plot(inhibition_force[0], inhibition_force[1], color="red")
    # ax.legend([user_force_line, inhibition_force_line],
    #                   ["Simulated user pushing force", "Force provided by the robot"],
    #                   loc='upper right', fontsize=12)
    # # plt.xlabel("t (seconds)", fontsize=12)
    ax.set_ylabel("Force: (N)", fontsize=18)
    ax.set_yticks(np.linspace(0, 50, 6))
    ax.set_xticks(np.linspace(0, 35, 8))
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    

    # Scenario A
    # ax.set_xticks(list(ax.get_xticks()))
    # ax.text(x=3.0, y=35, s="Inhibition force", color="red", fontsize=12)
    # ax.text(x=4.7, y=37, s=r"$f_{inh}$", color="red", fontsize=16)
    # ax.arrow(5, 35, -3.5, -25, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax.arrow(5, 35, -1, -12, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax.arrow(5, 35, 1.3, -17, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)

    # # ax.text(x=15.0, y=35, s="Estop force", color="gray", fontsize=12)
    # ax.text(x=15.5, y=37, s=r"$f_{estop}$", color="red", fontsize=16)
    # # ax.arrow(16, 35, -3.5, -8, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax.arrow(16, 35, -3, -12, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # # ax.arrow(16, 35, -2, -12, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax.arrow(16, 35, 8.0, -15, head_width=0.8, head_length=1.2, width=0.001, ec ='gray', alpha=0.5)


    # Scenario B
    # ax_list[0].set_xticks(list(ax_list[0].get_xticks()) + [16.32] + [])
    # # ax_list[0].text(x=14.0, y=35, s="Inhibition force", color="red", fontsize=12)
    # ax_list[0].text(x=15.5, y=37, s=r"$f_{inh}$", color="red", fontsize=16)
    # ax_list[0].arrow(16, 35, 0.2, -17, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # # ax_list[0].arrow(5, 35, -1, -12, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # # ax_list[0].arrow(5, 35, 1.3, -17, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)

    # # ax_list[0].text(x=10.0, y=35, s="Estop force", color="gray", fontsize=12)
    # ax_list[0].text(x=11.50, y=37, s=r"$f_{estop}$", color="red", fontsize=16)
    # ax_list[0].arrow(12, 35, -6.2, -12, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax_list[0].arrow(12, 35, -0.7, -5, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax_list[0].arrow(12, 35, 8.8, -10, head_width=0.8, head_length=1.2, width=0.001, ec ='gray', alpha=0.5)

    ax.set_ylim([-2.5, 50])
    ax.set_xlim([-2.5, 7.5])
    ax.grid(True, alpha=0.5)
    ax.set_xlabel("t (seconds)", fontsize=16)


    # ax = fig.add_subplot(413)
    # linear_velocity_line, = ax_list[1].plot(linear_velocity[0], linear_velocity[1])
    # plt.legend([linear_velocity_line], ["$v$"])
    # plt.xlabel("t (seconds)", fontsize=12)
    # ax_list[1].set_ylabel(r"$v$" + " : (m/s)", fontsize=14)
    # ax_list[1].set_yticks(np.linspace(-0.25, 1.0, 6))
    # ax_list[1].set_xticks(np.linspace(0, 35, 8))
    # ax_list[1].set_xlim([-2.5, 27.5])
    # ax_list[1].grid(True, alpha=0.5)

    # ax = fig.add_subplot(412)
    # system_torque_line, = ax_list[2].plot(system_torque[0], system_torque[1])
    # ax_list[2].legend([system_torque_line], ["System torque"])
    # plt.xlabel("t (seconds)", fontsize=12)
    # ax_list[2].set_ylabel("Driving torque:\n(N·m)", fontsize=14)
    # ax_list[2].set_yticks(np.linspace(-15, 15, 7))
    # ax_list[2].set_xticks(np.linspace(0, 35, 8))
    # ax_list[2].set_xlim([-2.5, 27.5])
    # ax_list[2].grid(True, alpha=0.5)

    # ax = fig.add_subplot(414)
    # angular_velocity_line, = ax_list[3].plot(angular_velocity[0], angular_velocity[1])
    # plt.legend([angular_velocity_line], ["$\omega$"])
    # ax_list[3].set_xlabel("t (seconds)", fontsize=12)
    # ax_list[3].set_ylabel(r"$\omega$" + " : (1/s)", fontsize=14)
    # ax_list[3].set_yticks(np.linspace(-0.75, 0.75, 7))
    # ax_list[3].set_xticks(np.linspace(0, 35, 8))
    # ax_list[3].set_xlim([-2.5, 27.5])
    # ax_list[3].grid(True, alpha=0.5)

    plt.subplots_adjust(top=0.96, bottom=0.18, left=0.08, right=0.97, hspace=0.3)
    plt.show()


def plot_force(data_dict):
    # fig = plt.figure(figsize=(12, 9))
    fig, ax_list = plt.subplots(4, 1, figsize=(12, 8), gridspec_kw={'height_ratios': [2.5, 1.5, 1.5, 1.5]})
    
    user_force = np.vstack((data_dict["/walker/force_filtered/wrench/force/y_x"],
                                 data_dict["/walker/force_filtered/wrench/force/y_y"]))
    inhibition_force = np.vstack((data_dict["/walker/inhibition_force/data_x"],
                                  data_dict["/walker/inhibition_force/data_y"]))
    system_torque = np.vstack((data_dict["/walker/system_torque/data_x"],
                               data_dict["/walker/system_torque/data_y"]))
    linear_velocity = np.vstack((data_dict["/walker/odom_filtered/twist/twist/linear/x_x"],
                                 data_dict["/walker/odom_filtered/twist/twist/linear/x_y"]))
    angular_velocity = np.vstack((data_dict["/walker/odom_filtered/twist/twist/angular/z_x"],
                                  data_dict["/walker/odom_filtered/twist/twist/angular/z_y"]))
    # Scenario A
    time_start = 4.29
    
    # Scenario B
    # time_start = 2.84


    user_force[0] -= time_start
    inhibition_force[0] -= time_start
    system_torque[0] -= time_start
    linear_velocity[0] -= time_start
    angular_velocity[0] -= time_start

    # ax = fig.add_subplot(411)
    user_force_line, = ax_list[0].plot(user_force[0], user_force[1])
    inhibition_force_line, = ax_list[0].plot(inhibition_force[0], inhibition_force[1], color="red")
    ax_list[0].legend([user_force_line, inhibition_force_line],
                      ["Simulated user pushing force", "Force provided by the robot"],
                      loc='upper right', fontsize=12)
    # plt.xlabel("t (seconds)", fontsize=12)
    ax_list[0].set_ylabel("Force: (N)", fontsize=14)
    ax_list[0].set_yticks(np.linspace(0, 50, 6))
    ax_list[0].set_xticks(np.linspace(0, 35, 8))

    # Scenario A
    ax_list[0].set_xticks(list(ax_list[0].get_xticks()) + [1.21, 3.74, 6.77] + [12.22])
    # ax_list[0].text(x=3.0, y=35, s="Inhibition force", color="red", fontsize=12)
    ax_list[0].text(x=4.7, y=37, s=r"$f_{inh}$", color="red", fontsize=16)
    ax_list[0].arrow(5, 35, -3.5, -25, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    ax_list[0].arrow(5, 35, -1, -12, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    ax_list[0].arrow(5, 35, 1.3, -17, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)

    # ax_list[0].text(x=15.0, y=35, s="Estop force", color="gray", fontsize=12)
    ax_list[0].text(x=15.5, y=37, s=r"$f_{estop}$", color="red", fontsize=16)
    # ax_list[0].arrow(16, 35, -3.5, -8, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    ax_list[0].arrow(16, 35, -3, -12, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax_list[0].arrow(16, 35, -2, -12, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    ax_list[0].arrow(16, 35, 8.0, -15, head_width=0.8, head_length=1.2, width=0.001, ec ='gray', alpha=0.5)


    # Scenario B
    # ax_list[0].set_xticks(list(ax_list[0].get_xticks()) + [16.32] + [])
    # # ax_list[0].text(x=14.0, y=35, s="Inhibition force", color="red", fontsize=12)
    # ax_list[0].text(x=15.5, y=37, s=r"$f_{inh}$", color="red", fontsize=16)
    # ax_list[0].arrow(16, 35, 0.2, -17, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # # ax_list[0].arrow(5, 35, -1, -12, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # # ax_list[0].arrow(5, 35, 1.3, -17, head_width=0.2, head_length=2, width=0.001, ec ='gray', alpha=0.5)

    # # ax_list[0].text(x=10.0, y=35, s="Estop force", color="gray", fontsize=12)
    # ax_list[0].text(x=11.50, y=37, s=r"$f_{estop}$", color="red", fontsize=16)
    # ax_list[0].arrow(12, 35, -6.2, -12, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax_list[0].arrow(12, 35, -0.7, -5, head_width=0.3, head_length=2, width=0.001, ec ='gray', alpha=0.5)
    # ax_list[0].arrow(12, 35, 8.8, -10, head_width=0.8, head_length=1.2, width=0.001, ec ='gray', alpha=0.5)

    ax_list[0].set_ylim([-2.5, 50])
    ax_list[0].set_xlim([-2.5, 27.5])
    ax_list[0].grid(True, alpha=0.5)


    # ax = fig.add_subplot(413)
    linear_velocity_line, = ax_list[1].plot(linear_velocity[0], linear_velocity[1])
    # plt.legend([linear_velocity_line], ["$v$"])
    # plt.xlabel("t (seconds)", fontsize=12)
    ax_list[1].set_ylabel(r"$v$" + " : (m/s)", fontsize=14)
    ax_list[1].set_yticks(np.linspace(-0.25, 1.0, 6))
    ax_list[1].set_xticks(np.linspace(0, 35, 8))
    ax_list[1].set_xlim([-2.5, 27.5])
    ax_list[1].grid(True, alpha=0.5)

    # ax = fig.add_subplot(412)
    system_torque_line, = ax_list[2].plot(system_torque[0], system_torque[1])
    # ax_list[2].legend([system_torque_line], ["System torque"])
    # plt.xlabel("t (seconds)", fontsize=12)
    ax_list[2].set_ylabel("Driving torque:\n(N·m)", fontsize=14)
    ax_list[2].set_yticks(np.linspace(-15, 15, 7))
    ax_list[2].set_xticks(np.linspace(0, 35, 8))
    ax_list[2].set_xlim([-2.5, 27.5])
    ax_list[2].grid(True, alpha=0.5)

    # ax = fig.add_subplot(414)
    angular_velocity_line, = ax_list[3].plot(angular_velocity[0], angular_velocity[1])
    # plt.legend([angular_velocity_line], ["$\omega$"])
    ax_list[3].set_xlabel("t (seconds)", fontsize=12)
    ax_list[3].set_ylabel(r"$\omega$" + " : (1/s)", fontsize=14)
    ax_list[3].set_yticks(np.linspace(-0.75, 0.75, 7))
    ax_list[3].set_xticks(np.linspace(0, 35, 8))
    ax_list[3].set_xlim([-2.5, 27.5])
    ax_list[3].grid(True, alpha=0.5)

    plt.subplots_adjust(top=0.98, bottom=0.08, left=0.08, right=0.97, hspace=0.3)
    plt.show()
'''


if __name__ == '__main__':
    # df = pd.read_csv("/home/samliu/code/socially-aware-walker/catkin_ws/aaaaa.csv")

    # Scenario A
    csvfile = open("/home/samliu/code/socially-aware-walker/catkin_ws/good_result3/robot_states.csv", "r")

    # Scenario B
    # csvfile = open("/home/samliu/code/socially-aware-walker/catkin_ws/good_result2/robot_states.csv", "r")

    dict_reader = csv.DictReader(csvfile)
    data_dict = {}
    for idx, row_dict in enumerate(dict_reader):
        if idx == 0:
            for key in row_dict.keys():
                if key is not None:
                    data_dict[key] = []
        for key in row_dict.keys():
            if key is not None:
                try:
                    value = Decimal(row_dict[key])
                    data_dict[key].append(value)
                except:
                    # Do nothing, skip this grid
                    pass
    for key in data_dict.keys():
        data_dict[key] = np.array(data_dict[key], dtype=np.float64)
    plot_force3(data_dict)
#!/usr/bin/env python3
import os
import argparse
import numpy as np
import csv
import re
import pandas as pd

import matplotlib.lines as mlines
from matplotlib import patches
from matplotlib.patches import Polygon

TIME_LIMIT = 35.0
METHOD_LIST = ["orca", "ComfortBasedAstar", "OriginalAGFAstar", "SociallyAwareAstar", "SARL", "LSTM"]
# METHOD_LIST = ["SociallyAwareAstar",]
HUMAN_RADIUS = 0.4
ROBOT_RADIUS = 0.7
TIME_STEP = 0.25


def my_rotate(pt, theta, offset_pt):
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                [np.sin(theta),  np.cos(theta)]],
                                dtype=np.float32)
    new_pt = np.dot(rotation_matrix, pt) + offset_pt
    return new_pt


def render(dataframe, mode='human', output_file=None):
    from matplotlib import animation
    import matplotlib.pyplot as plt
    plt.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg'

    x_offset = 0.11 * (-5)
    y_offset = 0.11
    cmap = plt.cm.get_cmap('Paired', 10)
    robot_color = 'black' # 'yellow'
    goal_color = 'red'
    arrow_color = 'red'
    arrow_style = patches.ArrowStyle("->", head_length=4, head_width=2)

    if mode == 'traj':
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.tick_params(labelsize=16)
        ax.set_xlim(-8, 8)
        ax.set_ylim(-6, 6)
        plt.gca().set_aspect("equal")
        ax.set_xlabel('x(m)', fontsize=16)
        ax.set_ylabel('y(m)', fontsize=16)

        ##### Plot the walls start #####
        ## Sim 2-1
        wall = plt.Line2D((-2, -2), (-6, -4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((2, 2), (-6, -4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)

        wall = plt.Line2D((-2, -4), (-4, -4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((2, 4), (-4, -4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((-4, -4), (-4, -2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((4, 4), (-4, -2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((-4, -4), (2, 4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((4, 4), (2, 4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((-2, -4), (4, 4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((2, 4), (4, 4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)

        wall = plt.Line2D((-4, -8), (-2, -2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((4, 8), (-2, -2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((-4, -8), (2, 2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((4, 8), (2, 2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)

        wall = plt.Line2D((-2, -2), (4, 6), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((2, 2), (4, 6), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)


        ## Sim 2-2
        # wall = plt.Line2D((-2, -2), (-6, 6), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        # wall = plt.Line2D((2, 2), (-6, 6), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        ##### Plot the walls end #####

        robot_positions = dataframe[["pxr", "pyr"]].to_numpy()
        agent_info_list = []
        num_agents = 0

        # add robot footprint
        robot_theta_list = dataframe["thetar"].to_numpy()
        polygon_pts = np.array([[0.45, 0.4],
                                [0.65, 0.2],
                                [0.65, -0.2],
                                [0.45, -0.4],
                                [-0.15, -0.4],
                                [-0.15, 0.4]])
        polygon_list = []
        for i in range(len(dataframe)):
            pts = polygon_pts.copy()
            for j, pt in enumerate(pts):
                pts[j] = my_rotate(pt, theta=robot_theta_list[i], offset_pt=robot_positions[i])
            polygon_list.append(pts)

        for i in range(1, 32):
            if ("px"+str(i)) in dataframe.columns:
                agent_info_list.extend([("px"+str(i)), ("py"+str(i))])
                num_agents += 1
            else: break
        human_positions = dataframe[agent_info_list].to_numpy().reshape((len(dataframe), num_agents, 2))
        for k in range(len(dataframe)):
            if k % 10 == 0 or k == len(dataframe) - 1:

                footprint = Polygon(polygon_list[k], facecolor='k', fill=False, alpha=np.tanh(float(k + 10) / len(dataframe)))
                ax.add_artist(footprint)

                robot = plt.Circle(robot_positions[k], ROBOT_RADIUS, fill=False, color=robot_color, alpha=np.tanh(float(k) / len(dataframe)))
                # ax.add_artist(robot)
                humans = [plt.Circle(human_positions[k][i], HUMAN_RADIUS, fill=False, color=cmap(i), alpha=(float(k) / len(dataframe)))
                          for i in range(num_agents)]
                for human in humans:
                    ax.add_artist(human)
            # add time annotation
            global_time = k * TIME_STEP
            if global_time % 5 == 0 or k == len(dataframe) - 1:
                # agents = humans + [robot]
                agents = [robot]
                if k == len(dataframe) - 1:
                    times = [plt.text(agents[i].center[0] - x_offset, agents[i].center[1] - y_offset,
                                      '{:.1f}'.format(global_time + 0.0),
                                      color='black', fontsize=14) for i in range(len(agents))]
                else:
                    times = [plt.text(agents[i].center[0] - x_offset, agents[i].center[1] - y_offset,
                                  '{:.1f}'.format(global_time),
                                  color='black', fontsize=14) for i in range(len(agents))]
                for time in times:
                    ax.add_artist(time)
            if k != 0:
                nav_direction = plt.Line2D((robot_positions[k - 1][0], robot_positions[k][0]),
                                           (robot_positions[k - 1][1], robot_positions[k][1]),
                                           color=robot_color, ls='solid', alpha=(float(k) / len(dataframe)))
                human_directions = [plt.Line2D((human_positions[k - 1][i][0], human_positions[k][i][0]),
                                               (human_positions[k - 1][i][1], human_positions[k][i][1]),
                                               color=cmap(i), ls='solid', alpha=(float(k) / len(dataframe)))
                                    for i in range(num_agents)]
                ax.add_artist(nav_direction)
                for human_direction in human_directions:
                    ax.add_artist(human_direction)
        # plt.legend([robot], ['Robot'], fontsize=12)
        plt.legend([footprint, wall], ['Robot', 'Wall'], fontsize=12)
        plt.show()

    elif mode == 'video':
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.tick_params(labelsize=16)
        ax.set_xlim(-8, 8)
        ax.set_ylim(-6, 6)
        plt.gca().set_aspect("equal")
        ax.set_xlabel('x(m)', fontsize=16)
        ax.set_ylabel('y(m)', fontsize=16)

        ##### Plot the walls start #####
        ## Sim 2-1
        wall = plt.Line2D((-2, -2), (-6, -4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((2, 2), (-6, -4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)

        wall = plt.Line2D((-2, -4), (-4, -4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((2, 4), (-4, -4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((-4, -4), (-4, -2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((4, 4), (-4, -2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((-4, -4), (2, 4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((4, 4), (2, 4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((-2, -4), (4, 4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((2, 4), (4, 4), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)

        wall = plt.Line2D((-4, -8), (-2, -2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((4, 8), (-2, -2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((-4, -8), (2, 2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((4, 8), (2, 2), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)

        wall = plt.Line2D((-2, -2), (4, 6), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        wall = plt.Line2D((2, 2), (4, 6), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)


        ## Sim 2-2
        # wall = plt.Line2D((-2, -2), (-6, 6), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        # wall = plt.Line2D((2, 2), (-6, 6), color="gray", ls='solid', linewidth=3); ax.add_artist(wall)
        ##### Plot the walls end #####


        # add robot and its goal
        robot_positions = dataframe[["pxr", "pyr"]].to_numpy()
        # goal = mlines.Line2D([0], [4], color=goal_color, marker='*', linestyle='None', markersize=15, label='Goal')
        # robot = plt.Circle(robot_positions[0], ROBOT_RADIUS, fill=True, color=robot_color)
        # ax.add_artist(robot)
        # ax.add_artist(goal)
        # plt.legend([robot, goal], ['Robot conservative radius', 'Goal'], fontsize=12)

        # add humans and their numbers
        agent_info_list = []
        num_agents = 0

        for i in range(1, 32):
            if ("px"+str(i)) in dataframe.columns:
                agent_info_list.extend([("px"+str(i)), ("py"+str(i))])
                num_agents += 1
            else: break
        human_positions = dataframe[agent_info_list].to_numpy().reshape((len(dataframe), num_agents, 2))
        
        
        # Little trick for sim2_1
        for j in range(len(dataframe)):
            for i in range(num_agents):
                human_positions[j][i][1] += 0.15
                human_positions[j][i][0] += 0.2



        humans = [plt.Circle(human_positions[0][i], HUMAN_RADIUS, fill=False, color=cmap(i))
                  for i in range(num_agents)]
        # human_numbers = [plt.text(humans[i].center[0] - x_offset, humans[i].center[1] - y_offset, str(i),
        #                           color='black', fontsize=12) for i in range(num_agents)]
        for i, human in enumerate(humans):
            ax.add_artist(human)
            # ax.add_artist(human_numbers[i])

        # add time annotation
        # time = plt.text(-1.6, 5, 'Time: {}'.format(0), fontsize=16)
        time = plt.text(-1.7, 5, 'Time: {:.1f}'.format(0), fontsize=16)
        ax.add_artist(time)

        # add robot arrow
        robot_theta_list = dataframe["thetar"].to_numpy()
        orientation = [(robot_positions[i], [robot_positions[i][0] + ROBOT_RADIUS * np.cos(robot_theta_list[i]),
                                             robot_positions[i][1] + ROBOT_RADIUS * np.sin(robot_theta_list[i])]) for i in range(len(dataframe))]
        orientations = [orientation]
        arrows = [patches.FancyArrowPatch(*orientation[0], color=arrow_color, arrowstyle=arrow_style)
                  for orientation in orientations]
        for arrow in arrows:
            ax.add_artist(arrow)

        # add robot footprint
        polygon_pts = np.array([[0.45, 0.4],
                                [0.65, 0.2],
                                [0.65, -0.2],
                                [0.45, -0.4],
                                [-0.15, -0.4],
                                [-0.15, 0.4]])
        polygon_list = []
        for i in range(len(dataframe)):
            pts = polygon_pts.copy()
            for j, pt in enumerate(pts):
                pts[j] = my_rotate(pt, theta=robot_theta_list[i], offset_pt=robot_positions[i])
            polygon_list.append(pts)
        footprint = Polygon(polygon_list[0], facecolor='k', fill=False)
        ax.add_artist(footprint)
        # plt.legend([footprint, robot, goal],
        #            ["Robot footprint", 'Assumed radius', 'Goal'],
        #            fontsize=10)

        global_step = 0
        def update(frame_num):
            nonlocal global_step
            nonlocal arrows
            global_step = frame_num

            # Robot position change
            # robot.center = robot_positions[frame_num]

            # Humans position change
            for i, human in enumerate(humans):
                human.center = human_positions[frame_num][i]
                # human_numbers[i].set_position((human.center[0] - x_offset, human.center[1] - y_offset))
            # Arrow change
            for arrow in arrows:
                arrow.remove()
            arrows = [patches.FancyArrowPatch(*orientation[frame_num], color=arrow_color,
                                              arrowstyle=arrow_style) for orientation in orientations]
            for arrow in arrows:
                ax.add_artist(arrow)

            footprint.set_xy(polygon_list[frame_num])

            time.set_text('Time: {:.1f}'.format(frame_num * TIME_STEP))


            # add robot path
            nav_direction = plt.Line2D((robot_positions[frame_num - 1][0], robot_positions[frame_num][0]),
                                       (robot_positions[frame_num - 1][1], robot_positions[frame_num][1]),
                                       color=robot_color, ls='solid', alpha=(float(frame_num) / len(dataframe)))
            ax.add_artist(nav_direction)

            human_directions = [plt.Line2D((human_positions[frame_num - 1][i][0], human_positions[frame_num][i][0]),
                                               (human_positions[frame_num - 1][i][1], human_positions[frame_num][i][1]),
                                               color=cmap(i), ls='solid', alpha=(float(frame_num) / len(dataframe)))
                                    for i in range(num_agents)]
            for human_direction in human_directions:
                    ax.add_artist(human_direction)

            # if frame_num >= len(dataframe) - 1:
            #     plt.close()


        def on_click(event):
            anim.running ^= True
            if anim.running:
                anim.event_source.stop()
                # if hasattr(self.robot.policy, 'action_values'):
                #     plot_value_heatmap()
            else:
                anim.event_source.start()

        fig.canvas.mpl_connect('key_press_event', on_click)
        anim = animation.FuncAnimation(fig, update, frames=len(dataframe), interval=TIME_STEP * 1000 / 2, repeat=False)
        anim.running = True

        if output_file is not None:
            ffmpeg_writer = animation.writers['ffmpeg']
            writer = ffmpeg_writer(fps=8, metadata=dict(artist='Me'), bitrate=1800)
            anim.save(output_file, writer=writer)
        else:
            plt.show()


if __name__ == '__main__':
    # df = pd.read_csv("/home/samliu/code/socially-aware-walker/catkin_ws/good_result2/SociallyAwareAstar_case0_h8_d8_v05_t20210930173931.csv")
    # render(df, "video", output_file="sim2_2_animation.gif")

    df = pd.read_csv("/home/samliu/code/socially-aware-walker/catkin_ws/good_result1/SociallyAwareAstar_case0_h8_d8_v05_t20210930114256.csv")
    render(df, "video", output_file="sim2_1_animation.gif")
    exit(-1)

    files_current_dir = os.listdir("testcase_h8_d8_v05")
    target_dir_list = []
    for dirname in files_current_dir:
        for method in METHOD_LIST:
            if method.lower() in dirname.lower(): target_dir_list.append(dirname)
    target_dir_list.sort()

    for target_dir in target_dir_list:
        method_dict = {}
        method_dict["name"] = target_dir.split("_")[0]
        target_dir = os.path.join("testcase_h8_d8_v05", target_dir)
        
        target_files = os.listdir(target_dir)
        target_files = [filename for filename in target_files if filename.endswith(".csv")]
        target_files.sort(key=lambda s: int(re.search(r'\d+', s).group(0)))
        # print(target_files)
        num_cases = len(target_files)

        elapsed_time_list = []
        min_dis_list = []
        collision_case_list = []
        timeout_case_list = []
        for idx, filename in enumerate(target_files):
            df = pd.read_csv(os.path.join(target_dir, filename))
            # render(df, "video")
            # exit(-1)
            if df["succ"].iloc[-1] == "yes":
                min_dis_list.append(min(df["min distance"])) # get min distance
                elapsed_time_list.append(df["time stamp"].iloc[-1])         # get elaspsed time
            else:
                if df["time stamp"].iloc[-1] < TIME_LIMIT:
                    collision_case_list.append(idx + 1)                     # count collision
                else:
                    timeout_case_list.append(idx + 1)                       # count timeout
     
        # elapsed_time_list = list(filter(lambda a: a < 100.0, elapsed_time_list))
        method_dict["succ_rate"] = len(elapsed_time_list) / num_cases
        method_dict["collision_rate"] = len(collision_case_list) / num_cases
        method_dict["timeout_rate"] = len(timeout_case_list) / num_cases
        method_dict["avg_navi_time"] = np.mean(elapsed_time_list)
        method_dict["75pctl_navi_time"] = np.percentile(elapsed_time_list, 75)
        method_dict["90pctl_navi_time"] = np.percentile(elapsed_time_list, 90)

        method_dict["avg_min_dis"] = np.mean(min_dis_list)
        method_dict["25pctl_min_dis"] = np.percentile(min_dis_list, 25)
        method_dict["10pctl_min_dis"] = np.percentile(min_dis_list, 10)

        print("====================")
        print("{}:".format(method_dict["name"]))
        # print("    Succ rate\t\t: {:.1f} %".format(method_dict["succ_rate"] * 100))
        print("    Collision rate\t: {:.1f} %".format(method_dict["collision_rate"] * 100))
        print("    Timeout rate\t: {:.1f} %".format(method_dict["timeout_rate"] * 100))
        # print("    Avg_navi_time\t: {:.2f} secs".format(method_dict["avg_navi_time"]))
        # print("    75pctl_navi_time\t: {:.3f} secs".format(method_dict["75pctl_navi_time"]))
        # print("    90pctl_navi_time\t: {:.2f} secs".format(method_dict["90pctl_navi_time"]))
        print("  elapsed_time \t: ({:.2f} / {:.2f}) secs".format(method_dict["avg_navi_time"], method_dict["90pctl_navi_time"]))

        # print("    avg_min_dis\t\t: {:.2f} m".format(method_dict["avg_min_dis"]))
        # print("    25pctl_min_dis\t: {:.3f} m".format(method_dict["25pctl_min_dis"]))
        # print("    10pctl_min_dis\t: {:.2f} m".format(method_dict["10pctl_min_dis"]))
        print(" min_dis\t: ({:.2f} / {:.2f}) m".format(method_dict["avg_min_dis"], method_dict["10pctl_min_dis"]))
        print(" rate\t: {:.1f} ({:.1f} / {:.1f}) %".format(method_dict["collision_rate"] * 100 + method_dict["timeout_rate"] * 100,
                                                           method_dict["collision_rate"] * 100,
                                                           method_dict["timeout_rate"] * 100))

        # print("    Collision cases:\t:", collision_case_list)
        # print("    Timeout cases:\t:", timeout_case_list)
        print()

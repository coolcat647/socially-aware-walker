#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.text import Annotation
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.proj3d import proj_transform

# Extend Annotation3D
class Annotation3D(Annotation):
    def __init__(self, text, xyz, *args, **kwargs):
        super().__init__(text, xy=(0, 0), *args, **kwargs)
        self._xyz = xyz
    def draw(self, renderer):
        x2, y2, z2 = proj_transform(*self._xyz, self.axes.M)
        self.xy = (x2, y2)
        super().draw(renderer)
def _annotate3D(ax, text, xyz, *args, **kwargs):
    '''Add anotation `text` to an `Axes3d` instance.'''
    annotation = Annotation3D(text, xyz, *args, **kwargs)
    ax.add_artist(annotation)
setattr(Axes3D, 'annotate3D', _annotate3D)

class Arrow3D(FancyArrowPatch):
    def __init__(self, x, y, z, dx, dy, dz, *args, **kwargs):
        super().__init__((0, 0), (0, 0), *args, **kwargs)
        self._xyz = (x, y, z)
        self._dxdydz = (dx, dy, dz)
    def draw(self, renderer):
        x1, y1, z1 = self._xyz
        dx, dy, dz = self._dxdydz
        x2, y2, z2 = (x1 + dx, y1 + dy, z1 + dz)
        xs, ys, zs = proj_transform((x1, x2), (y1, y2), (z1, z2), self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        super().draw(renderer)
def _arrow3D(ax, x, y, z, dx, dy, dz, *args, **kwargs):
    '''Add an 3d arrow to an `Axes3D` instance.'''
    arrow = Arrow3D(x, y, z, dx, dy, dz, *args, **kwargs)
    ax.add_artist(arrow)
setattr(Axes3D, 'arrow3D', _arrow3D)


fig = plt.figure(figsize=(16, 8))
figure = plt.gca()
x_axis = figure.axes.get_xaxis()
x_axis.set_visible(False)
y_axis = figure.axes.get_yaxis()
y_axis.set_visible(False)
plt.box(False)

resolution = 0.2
speed = 0.5
x_middle, y_middle = (0.0, 0.0)

def show_social_agf():
    fig.set_size_inches(8, 8)
    figure = plt.gca()
    x_axis = figure.axes.get_xaxis()
    x_axis.set_visible(False)
    y_axis = figure.axes.get_yaxis()
    y_axis.set_visible(False)
    plt.cla()

    yaw = np.pi * 7 / 4
    t = np.arange(-4, 4 + resolution, resolution)
    x, y = np.meshgrid(t, t)
    sigma_head = np.max([speed, 0.5])
    sigma_right = sigma_head * 3 / 5
    sigma_left = sigma_head * 1 / 5
    sigma_rear = sigma_head / 2
    alpha = np.arctan2(y_middle - y, x_middle - x) - yaw - np.pi * 0.5
    alpha_normalized = np.arctan2(np.sin(alpha), np.cos(alpha))
    sigma_front = (alpha_normalized > np.zeros(alpha_normalized.shape)) * sigma_head + (alpha_normalized <= np.zeros(alpha_normalized.shape)) * sigma_rear
        
    alpha_side = np.arctan2(np.sin(alpha + np.pi * 0.5), np.cos(alpha + np.pi * 0.5))
    sigma_side = ((alpha_side) > np.zeros(alpha_side.shape)) * sigma_right + ((alpha_side) <= np.zeros(alpha_side.shape)) * sigma_left

    g_a = np.power(np.cos(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.sin(yaw), 2) / (2 * np.power(sigma_side, 2))
    g_b = np.sin(2 * yaw) / (4 * np.power(sigma_front, 2)) - np.sin(2 * yaw) / (4 * np.power(sigma_side, 2))
    g_c = np.power(np.sin(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.cos(yaw), 2) / (2 * np.power(sigma_side, 2))
    z = 1 / np.exp(g_a * np.power(x_middle - x, 2) + 2 * g_b * (x_middle - x) * (y_middle - y) + g_c * np.power(y_middle - y, 2))
    z = np.clip(z, 1e-9, 1)

    ax = Axes3D(fig)
    ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
    ax.contour(x, y, z, offset=1.1, cmap='rainbow')
    ax.scatter(x_middle, y_middle, 1.1, marker='o', c='red')
    ax.set_xlabel("x", fontsize=12, fontweight='bold')
    ax.set_ylabel("y", fontsize=12, fontweight='bold')
    ax.set_zlabel("z", fontsize=12, fontweight='bold')
    ax.set_title("Socially-aware AGF")
    plt.show()


'''
    Animation direction
'''
def show_animation_direction():
    for yaw in np.arange(0.0, np.pi * 2, np.pi / 4):
        plt.cla()
        
        # Press "ESC" for stopping simulation
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
        

        t = np.arange(-4, 4 + resolution, resolution)
        x, y = np.meshgrid(t, t)

        # Original AGF
        sigma_head = np.max([speed, 0.5])
        sigma_side = sigma_head * 2 / 3
        sigma_rear = sigma_head / 2
        alpha = np.arctan2(y_middle - y, x_middle - x) - yaw - np.pi * 0.5
        alpha_normalized = np.arctan2(np.sin(alpha), np.cos(alpha))
        sigma_front = (alpha_normalized > np.zeros(alpha_normalized.shape)) * sigma_head + (alpha_normalized <= np.zeros(alpha_normalized.shape)) * sigma_rear
        
        g_a = np.power(np.cos(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.sin(yaw), 2) / (2 * np.power(sigma_side, 2))
        g_b = np.sin(2 * yaw) / (4 * np.power(sigma_front, 2)) - np.sin(2 * yaw) / (4 * np.power(sigma_side, 2))
        g_c = np.power(np.sin(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.cos(yaw), 2) / (2 * np.power(sigma_side, 2))
        z = 1 / np.exp(g_a * np.power(x_middle - x, 2) + 2 * g_b * (x_middle - x) * (y_middle - y) + g_c * np.power(y_middle - y, 2))

        ax = fig.add_subplot(1, 2, 1, projection='3d')
        ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
        ax.contour(x, y, z, offset=1.1, cmap='rainbow')
        ax.scatter(x_middle, y_middle, 1.1, marker='o', c='red')
        ax.set_xlabel("x", fontsize=12, fontweight='bold')
        ax.set_ylabel("y", fontsize=12, fontweight='bold')
        ax.set_zlabel("z", fontsize=12, fontweight='bold')
        ax.set_title("Original AGF")

        
        # Socially-aware AGF
        sigma_right = sigma_head * 3 / 5
        sigma_left = sigma_head * 1 / 5
        alpha_side = np.arctan2(np.sin(alpha + np.pi * 0.5), np.cos(alpha + np.pi * 0.5))
        sigma_side = ((alpha_side) > np.zeros(alpha_side.shape)) * sigma_right + ((alpha_side) <= np.zeros(alpha_side.shape)) * sigma_left

        g_a = np.power(np.cos(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.sin(yaw), 2) / (2 * np.power(sigma_side, 2))
        g_b = np.sin(2 * yaw) / (4 * np.power(sigma_front, 2)) - np.sin(2 * yaw) / (4 * np.power(sigma_side, 2))
        g_c = np.power(np.sin(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.cos(yaw), 2) / (2 * np.power(sigma_side, 2))
        z = 1 / np.exp(g_a * np.power(x_middle - x, 2) + 2 * g_b * (x_middle - x) * (y_middle - y) + g_c * np.power(y_middle - y, 2))
        z = np.clip(z, 1e-9, 1)

        ax = fig.add_subplot(1, 2, 2, projection='3d')
        ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
        ax.contour(x, y, z, offset=1.1, cmap='rainbow')
        ax.scatter(x_middle, y_middle, 1.1, marker='o', c='red')
        ax.set_xlabel("x", fontsize=12, fontweight='bold')
        ax.set_ylabel("y", fontsize=12, fontweight='bold')
        ax.set_zlabel("z", fontsize=12, fontweight='bold')
        ax.set_title("Socially-aware AGF")
        fig.tight_layout()
        # plt.subplots_adjust(left=None, bottom=None, right=None, top=0.5, wspace=0.01, hspace=None)

        plt.pause(0.1)
    plt.show()


def show_comparison(flag_display_3d=False):
    resolution = 0.1

    SPEED_RANGE = 2
    XY_LIMIT_RANGE = 4
    fig.set_size_inches(8, SPEED_RANGE * 3)

    for cnt_plot in range(SPEED_RANGE):
        speed = (cnt_plot + 1) * 0.5
        yaw = np.pi * 7 / 4

        t = np.arange(-XY_LIMIT_RANGE, XY_LIMIT_RANGE + resolution, resolution)
        x, y = np.meshgrid(t, t)

        # Original AGF
        sigma_head = np.max([speed * 2, 1.0])
        sigma_side = sigma_head * 2 / 3
        sigma_rear = sigma_head / 2
        # sigma_rear = sigma_head / 2 * (2.0 - np.min([speed, 1.5])) / 1.5
        alpha = np.arctan2(y - y_middle, x - x_middle) - yaw + np.pi * 0.5
        alpha_normalized = np.arctan2(np.sin(alpha), np.cos(alpha))
        sigma_front = (alpha_normalized > np.zeros(alpha_normalized.shape)) * sigma_head + (alpha_normalized <= np.zeros(alpha_normalized.shape)) * sigma_rear
        
        g_a = np.power(np.cos(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.sin(yaw), 2) / (2 * np.power(sigma_side, 2))
        g_b = np.sin(2 * yaw) / (4 * np.power(sigma_front, 2)) - np.sin(2 * yaw) / (4 * np.power(sigma_side, 2))
        g_c = np.power(np.sin(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.cos(yaw), 2) / (2 * np.power(sigma_side, 2))
        z = 1 / np.exp(g_a * np.power(x_middle - x, 2) + 2 * g_b * (x_middle - x) * (y_middle - y) + g_c * np.power(y_middle - y, 2))

        # Measure the minimum safe distance
        val_xy_cross0_8 = 0
        for val_xy in np.arange(0, XY_LIMIT_RANGE + resolution, resolution):
            idx_x = int((val_xy - (-XY_LIMIT_RANGE)) / resolution)
            idx_y = int((-val_xy - (-XY_LIMIT_RANGE)) / resolution)
            if z[idx_y, idx_x] < 0.8:
                val_xy_cross0_8 = val_xy - resolution
                break

        # Plot 3D
        if flag_display_3d:
            ax = fig.add_subplot(SPEED_RANGE, 2, 1 + cnt_plot * 2, projection='3d')
            ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
            ax.contour(x, y, z, levels=np.linspace(0, 1, 6), offset=1.1, cmap='rainbow')
            ax.arrow3D(x_middle, y_middle, 1.1,
                   val_xy_cross0_8, -val_xy_cross0_8, 0.0,
                   mutation_scale=8,
                   arrowstyle="-",
                   linestyle='dashed')
            ax.text2D(0.60, 0.55, "{:.2f} m".format(np.hypot(val_xy_cross0_8, -val_xy_cross0_8)), transform=ax.transAxes)
            ax.set_xticks(np.linspace(-XY_LIMIT_RANGE, XY_LIMIT_RANGE, 5))
            ax.set_yticks(np.linspace(-XY_LIMIT_RANGE, XY_LIMIT_RANGE, 5))

        # Plot 2D
        else:
            ax = fig.add_subplot(SPEED_RANGE, 2, 1 + cnt_plot * 2)
            ax.grid("on")
            plt_contour = ax.contour(x, y, z, levels=np.linspace(0, 1, 6), cmap='jet')
            # fig.colorbar(plt_contour, ax=ax)
            ax.plot([x_middle, val_xy_cross0_8], [y_middle, -val_xy_cross0_8], linestyle="--", color="black")
            ax.text(0.6, 0.6, "{:.2f} m".format(np.hypot(val_xy_cross0_8, -val_xy_cross0_8)))


        ax.scatter(x_middle, y_middle, 1.1, marker='o', c='red')
        # ax.set_xlabel("x", fontsize=12, fontweight='bold')
        # ax.set_ylabel("y", fontsize=12, fontweight='bold')
        # ax.set_zlabel("z", fontsize=12, fontweight='bold')
        if speed <= 0.5:
            ax.set_title("AGF, $v_{target}\leq$" + "{:.1f} m/s".format(0.5))
        else:
            ax.set_title("AGF, $v_{target}$=" + "{:.1f} m/s".format(speed))

        
        # Socially-aware AGF
        sigma_right = sigma_head * 3 / 5
        sigma_left = sigma_head * 1 / 5
        alpha_side = np.arctan2(np.sin(alpha + np.pi * 0.5), np.cos(alpha + np.pi * 0.5))
        sigma_side = ((alpha_side) > np.zeros(alpha_side.shape)) * sigma_right + ((alpha_side) <= np.zeros(alpha_side.shape)) * sigma_left

        g_a = np.power(np.cos(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.sin(yaw), 2) / (2 * np.power(sigma_side, 2))
        g_b = np.sin(2 * yaw) / (4 * np.power(sigma_front, 2)) - np.sin(2 * yaw) / (4 * np.power(sigma_side, 2))
        g_c = np.power(np.sin(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.cos(yaw), 2) / (2 * np.power(sigma_side, 2))
        z = 1 / np.exp(g_a * np.power(x_middle - x, 2) + 2 * g_b * (x_middle - x) * (y_middle - y) + g_c * np.power(y_middle - y, 2))
        z = np.clip(z, 1e-9, 1)

        # Measure the minimum safe distance
        val_xy_cross0_8 = 0
        for val_xy in np.arange(0, XY_LIMIT_RANGE + resolution, resolution):
            idx_x = int((val_xy - (-XY_LIMIT_RANGE)) / resolution)
            idx_y = int((-val_xy - (-XY_LIMIT_RANGE)) / resolution)
            if z[idx_y, idx_x] < 0.8:
                val_xy_cross0_8 = val_xy - resolution
                # print("{:.2f}, {:.2f}".format(z[idx_y, idx_x], z[int((-val_xy_cross0_8 - (-XY_LIMIT_RANGE)) / resolution), int((val_xy_cross0_8 - (-XY_LIMIT_RANGE)) / resolution)]))
                break

        # Plot 3D
        if flag_display_3d:
            ax = fig.add_subplot(SPEED_RANGE, 2, 2 + cnt_plot * 2, projection='3d')
            ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
            plt_contour = ax.contour(x, y, z, levels=np.linspace(0, 1, 6), offset=1.1, cmap='rainbow')
            if cnt_plot == SPEED_RANGE - 1:
                fig.colorbar(plt_contour, ax=ax, fraction=0.02, pad=0.06, aspect=20)
            ax.arrow3D(x_middle, y_middle, 1.1,
                   val_xy_cross0_8, -val_xy_cross0_8, 0.0,
                   mutation_scale=8,
                   arrowstyle="-",
                   linestyle='dashed')
            ax.text2D(0.6, 0.6, "{:.2f} m".format(np.hypot(val_xy_cross0_8, -val_xy_cross0_8)), transform=ax.transAxes)
            ax.set_xticks(np.linspace(-XY_LIMIT_RANGE, XY_LIMIT_RANGE, 5))
            ax.set_yticks(np.linspace(-XY_LIMIT_RANGE, XY_LIMIT_RANGE, 5))

        # Plot 2D
        else:
            ax = fig.add_subplot(SPEED_RANGE, 2, 2 + cnt_plot * 2)
            ax.grid("on")
            plt_contour = ax.contour(x, y, z, levels=np.linspace(0, 1, 6), cmap='jet')
            if cnt_plot == SPEED_RANGE - 1:
                fig.colorbar(plt_contour, ax=ax)
            ax.plot([x_middle, val_xy_cross0_8], [y_middle, -val_xy_cross0_8], linestyle="--", color="black")
            ax.text(0.6, 0.6, "{:.2f} m".format(np.hypot(val_xy_cross0_8, -val_xy_cross0_8)))

        ax.scatter(x_middle, y_middle, 1.1, marker='o', c='red')
        if cnt_plot == SPEED_RANGE - 1:
            ax.set_xlabel("x", fontsize=12, fontweight='bold',labelpad=1)
            ax.set_ylabel("y", fontsize=12, fontweight='bold',labelpad=1)
            if flag_display_3d:
                ax.set_zlabel("z", fontsize=12, fontweight='bold',labelpad=1)
        if speed <= 0.5:
            ax.set_title("Our proposed method, $v_{target}\leq$" + "{:.1f} m/s".format(0.5))
        else:
            ax.set_title("Our proposed method, $v_{target}$=" + "{:.1f} m/s".format(speed))

        
        


        # if cnt_plot < SPEED_RANGE - 1:
        #     ax.axes.xaxis.set_ticklabels([])
        #     ax.axes.yaxis.set_ticklabels([])
        #     ax.axes.zaxis.set_ticklabels([])

        if flag_display_3d:
            plt.subplots_adjust(left=0.0, bottom=0.05, right=0.95, top=0.95, wspace=0.0, hspace=0.15)
        else:
            fig.tight_layout()


        
    plt.show()


'''
    Animation speed
'''
def show_animation_speed():
    for speed in np.arange(0.4, 1.1, 0.1):
        yaw = np.pi * 7 / 4
    # for yaw in np.arange(0.0, np.pi * 2, np.pi / 4):
        plt.cla()
        
        # Press "ESC" for stopping simulation
        plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
        

        t = np.arange(-4, 4 + resolution, resolution)
        x, y = np.meshgrid(t, t)

        # Original AGF
        sigma_head = np.max([speed, 0.5])
        sigma_side = sigma_head * 2 / 3
        sigma_rear = sigma_head / 2 * (2.0 - np.min([speed, 1.5])) / 1.5
        alpha = np.arctan2(y_middle - y, x_middle - x) - yaw - np.pi * 0.5
        alpha_normalized = np.arctan2(np.sin(alpha), np.cos(alpha))
        sigma_front = (alpha_normalized > np.zeros(alpha_normalized.shape)) * sigma_head + (alpha_normalized <= np.zeros(alpha_normalized.shape)) * sigma_rear
        
        g_a = np.power(np.cos(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.sin(yaw), 2) / (2 * np.power(sigma_side, 2))
        g_b = np.sin(2 * yaw) / (4 * np.power(sigma_front, 2)) - np.sin(2 * yaw) / (4 * np.power(sigma_side, 2))
        g_c = np.power(np.sin(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.cos(yaw), 2) / (2 * np.power(sigma_side, 2))
        z = 1 / np.exp(g_a * np.power(x_middle - x, 2) + 2 * g_b * (x_middle - x) * (y_middle - y) + g_c * np.power(y_middle - y, 2))

        ax = fig.add_subplot(1, 2, 1, projection='3d')
        ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
        ax.contour(x, y, z, offset=1.1, cmap='rainbow')
        ax.scatter(x_middle, y_middle, 1.1, marker='o', c='red')
        ax.set_xlabel("x", fontsize=12, fontweight='bold')
        ax.set_ylabel("y", fontsize=12, fontweight='bold')
        ax.set_zlabel("z", fontsize=12, fontweight='bold')
        ax.set_title("AGF, $v_{target}$=" + "{:.1f} m/s".format(speed))

        
        # Socially-aware AGF
        sigma_right = sigma_head * 3 / 5
        sigma_left = sigma_head * 1 / 5
        alpha_side = np.arctan2(np.sin(alpha + np.pi * 0.5), np.cos(alpha + np.pi * 0.5))
        sigma_side = ((alpha_side) > np.zeros(alpha_side.shape)) * sigma_right + ((alpha_side) <= np.zeros(alpha_side.shape)) * sigma_left

        g_a = np.power(np.cos(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.sin(yaw), 2) / (2 * np.power(sigma_side, 2))
        g_b = np.sin(2 * yaw) / (4 * np.power(sigma_front, 2)) - np.sin(2 * yaw) / (4 * np.power(sigma_side, 2))
        g_c = np.power(np.sin(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.cos(yaw), 2) / (2 * np.power(sigma_side, 2))
        z = 1 / np.exp(g_a * np.power(x_middle - x, 2) + 2 * g_b * (x_middle - x) * (y_middle - y) + g_c * np.power(y_middle - y, 2))
        z = np.clip(z, 1e-9, 1)

        ax = fig.add_subplot(1, 2, 2, projection='3d')
        ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
        ax.contour(x, y, z, offset=1.1, cmap='rainbow')
        ax.scatter(x_middle, y_middle, 1.1, marker='o', c='red')
        ax.set_xlabel("x", fontsize=12, fontweight='bold')
        ax.set_ylabel("y", fontsize=12, fontweight='bold')
        ax.set_zlabel("z", fontsize=12, fontweight='bold')
        ax.set_title("Our proposed method, $v_{target}$=" + "{:.1f} m/s".format(speed))
        fig.tight_layout()
        # plt.subplots_adjust(left=None, bottom=None, right=None, top=0.5, wspace=0.01, hspace=None)

        plt.pause(0.1)
    plt.show()


'''
    Butterworth filter
'''
def show_butterworth():
    fig.set_size_inches(8, 8)

    resolution = 0.1
    XY_LIMIT_RANGE = 1
    INFLATION_RADIUS = 0.2
    FILTER_ORDER = int(2)

    sigma_x = 1.0
    sigma_y = 1.0

    t = np.arange(-XY_LIMIT_RANGE, XY_LIMIT_RANGE + resolution, resolution)
    x, y = np.meshgrid(t, t)
    r = np.sqrt(x**2 + y**2)
    z = 1 / np.sqrt(1 + (r / INFLATION_RADIUS)**(2 * FILTER_ORDER))

    ax = Axes3D(fig)
    ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
    ax.contour(x, y, z, levels=np.linspace(0, 1, 6), offset=1.1, cmap='rainbow')
    ax.set_xlabel("x", fontsize=12, fontweight='bold',labelpad=1)
    ax.set_ylabel("y", fontsize=12, fontweight='bold',labelpad=1)
    ax.set_zlabel("z", fontsize=12, fontweight='bold',labelpad=1)
    ax.set_title(str(FILTER_ORDER) + "th-order Butterworth filter, $r_{inflation}$=" + "{:.1f} m".format(INFLATION_RADIUS))

    # Measure the minimum safe distance
    val_xy_cross0_8 = 0
    for val_xy in np.arange(0, XY_LIMIT_RANGE + resolution, resolution):
        idx_x = int((val_xy - (-XY_LIMIT_RANGE)) / resolution)
        idx_y = int((-val_xy - (-XY_LIMIT_RANGE)) / resolution)
        if z[idx_y, idx_x] < 0.6:
            val_xy_cross0_8 = val_xy - resolution
            # print("{:.2f}, {:.2f}".format(z[idx_y, idx_x], z[int((-val_xy_cross0_8 - (-XY_LIMIT_RANGE)) / resolution), int((val_xy_cross0_8 - (-XY_LIMIT_RANGE)) / resolution)]))
            break
    ax.arrow3D(x_middle, y_middle, 1.1,
               val_xy_cross0_8, -val_xy_cross0_8, 0.0,
               mutation_scale=8,
               arrowstyle="-",
               linestyle='dashed')
    ax.text2D(0.55, 0.75, "{:.2f} m".format(np.hypot(val_xy_cross0_8, -val_xy_cross0_8)), transform=ax.transAxes)

    plt.show()


if __name__ == '__main__':

    parser = argparse.ArgumentParser('Parse configuration file')
    # parser.add_argument('--animation', default=False, action='store_true')
    parser.add_argument('--case', type=int, default=0)
    # parser.add_argument('__name', type=str, default='rosnode_name')     # Dummy args for ROS
    # parser.add_argument('__log', type=str, default='log_file')      # Dummy args for ROS
    args = parser.parse_args()

    if args.case == 0:
        show_social_agf()
    elif args.case == 1:
        show_animation_direction()
    elif args.case == 2:
        show_animation_speed()
    elif args.case == 3:
        show_comparison(flag_display_3d=True)
    elif args.case == 4:
        show_comparison(flag_display_3d=False)
    elif args.case == 5:
        show_butterworth()
    else:
        raise KeyError("Unkown show case number: " + str(args.case))
        
        
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure(figsize=(16, 9))
figure = plt.gca()
x_axis = figure.axes.get_xaxis()
x_axis.set_visible(False)
y_axis = figure.axes.get_yaxis()
y_axis.set_visible(False)


'''
    Animation
'''

resolution = 0.8
speed = 1.6
x_middle, y_middle = (0.0, 0.0)

for yaw in np.arange(0.0, np.pi * 2, np.pi / 4):
    plt.cla()
    
    # Press "ESC" for stopping simulation
    plt.gcf().canvas.mpl_connect('key_release_event', lambda event: [exit(0) if event.key == 'escape' else None])
    

    t = np.arange(-8, 8 + resolution, resolution)
    x, y = np.meshgrid(t, t)

    # Original AGF
    sigma_head = np.max([speed * 2, 0.5])
    sigma_side = sigma_head * 2 / 3
    sigma_rear = sigma_head / 2
    alpha = np.arctan2(y_middle - y, x_middle - x) - yaw - np.pi * 0.5
    alpha_prime = np.arctan2(np.sin(alpha), np.cos(alpha))
    sigma_front = (alpha_prime > np.zeros(alpha_prime.shape)) * sigma_head + (alpha_prime <= np.zeros(alpha_prime.shape)) * sigma_rear
    
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
    sigma_side = ((alpha_side) >= np.zeros(alpha_side.shape)) * sigma_right + ((alpha_side) < np.zeros(alpha_side.shape)) * sigma_left

    g_a = np.power(np.cos(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.sin(yaw), 2) / (2 * np.power(sigma_side, 2))
    g_b = np.sin(2 * yaw) / (4 * np.power(sigma_front, 2)) - np.sin(2 * yaw) / (4 * np.power(sigma_side, 2))
    g_c = np.power(np.sin(yaw), 2) / (2 * np.power(sigma_front, 2)) + np.power(np.cos(yaw), 2) / (2 * np.power(sigma_side, 2))
    z = 1 / np.exp(g_a * np.power(x_middle - x, 2) + 2 * g_b * (x_middle - x) * (y_middle - y) + g_c * np.power(y_middle - y, 2))

    ax = fig.add_subplot(1, 2, 2, projection='3d')
    ax.plot_surface(x, y, z, rstride=1, cstride=1, cmap=plt.get_cmap('rainbow'))
    ax.contour(x, y, z, offset=1.1, cmap='rainbow')
    ax.scatter(x_middle, y_middle, 1.1, marker='o', c='red')
    ax.set_xlabel("x", fontsize=12, fontweight='bold')
    ax.set_ylabel("y", fontsize=12, fontweight='bold')
    ax.set_zlabel("z", fontsize=12, fontweight='bold')
    ax.set_title("Socially-aware AGF")

    plt.pause(0.1)

plt.show()
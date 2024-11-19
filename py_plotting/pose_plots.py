#!/usr/bin/env python3

"""
File:   pose_plots.py
Author: psherman-uva
Date:   July 2024
"""
from util_functions import *

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

plt.rcParams['lines.linewidth'] = 0.8
plt.rcParams['font.size']       = 5
plt.rcParams['text.usetex']     = True

DbFilename = "/home/patrick/uva/data/ConsensusProject.db"
DbTable    = "DebugTable"
ClrMap  = plt.get_cmap('Set1').colors

def get_data():
    sql_cmd = f"SELECT * FROM {DbTable}"
    return read_data_from_database(DbFilename, sql_cmd)


def robot_pose_plot(data):
    fig = plt.figure(layout='constrained', dpi=200)
    fig.set_size_inches(5.0, 4.5)
    gs = GridSpec(2, 1, figure=fig)

    ax1 = fig.add_subplot(gs[0])
    ax1.set_title("Pose (x)")
    for i in range(4):
        ax1.plot(data["idx"], data[f"x{i}"], color=ClrMap[i])
    ax1.set_xlabel("$t_k$")
    ax1.set_ylabel("$x$", rotation=0)
    ax1.legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])

    ax2 = fig.add_subplot(gs[1])
    ax2.set_title("Pose (y)")
    for i in range(4):
        ax2.plot(data["idx"], data[f"y{i}"], color=ClrMap[i])
    ax2.set_xlabel("$t_k$")
    ax2.set_ylabel("$y$", rotation=0)
    ax2.legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])


def robot_XY(data):
    fig = plt.figure(layout='constrained', dpi=200)
    fig.set_size_inches(5.0, 4.5)

    ax1 = fig.add_subplot()
    ax1.set_title("Robot Trajectory")
    for i in range(4):
        ax1.plot(data[f"x{i}"], data[f"y{i}"], "o", color=ClrMap[i])
    ax1.set_xlabel("$X$")
    ax1.set_ylabel("$Y$", rotation=0)
    ax1.set(xlim=[-2.5, 2.5], ylim=[-2.5, 2.5])
    ax1.legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])

def main():
    data = get_data()

    robot_XY(data)

    plt.show()

if __name__ == "__main__":
    main()
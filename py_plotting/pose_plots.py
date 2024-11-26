#!/usr/bin/env python3

"""
File:   pose_plots.py
Author: psherman-uva
Date:   July 2024
"""

import sys
sys.path.insert(1, "/home/patrick/uva/ros/catkin_ws/src/amrl_consensus/py_plotting")

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

from util_functions import *


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


def formation_center_plot(data):
    fig = plt.figure(layout='constrained', dpi=200)
    fig.set_size_inches(5.0, 4.5)
    gs = GridSpec(2, 1, figure=fig)

    ax1 = fig.add_subplot(gs[0])
    ax1.set_title("Pose (x)")
    for i in range(4):
        ax1.plot(data["idx"], data[f"Fx{i}"], color=ClrMap[i])
    ax1.set_xlabel("$t_k$")
    ax1.set_ylabel("$x$", rotation=0)
    ax1.legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])

    ax2 = fig.add_subplot(gs[1])
    ax2.set_title("Pose (y)")
    for i in range(4):
        ax2.plot(data["idx"], data[f"Fy{i}"], color=ClrMap[i])
    ax2.set_xlabel("$t_k$")
    ax2.set_ylabel("$y$", rotation=0)
    ax2.legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])


def consensus_plot(data):
    fig = plt.figure(layout='constrained', dpi=200)
    fig.set_size_inches(5.5, 4.5)
    gs = GridSpec(3, 1, figure=fig)

    ax1 = fig.add_subplot(gs[0])
    ax1.set_title("$\\xi(x)$")
    for i in range(4):
        ax1.plot(data["time"], data[f"xi{i}"], color=ClrMap[i])
    ax1.set_xlabel("$t_k\\ [s]$")
    ax1.set_ylabel("$x\\ [m]$", rotation=0)
    ax1.legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])

    ax2 = fig.add_subplot(gs[1])
    ax2.set_title("$\\xi(y)$")
    for i in range(4, 8):
        ax2.plot(data["time"], data[f"xi{i}"], color=ClrMap[i-4])
    ax2.set_xlabel("$t_k\\ [s]$")
    ax2.set_ylabel("$y\\ [m]$", rotation=0)
    ax2.legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])

    ax3 = fig.add_subplot(gs[2])
    ax3.set_title("$\\xi(z)$")
    for i in range(8, 12):
        ax3.plot(data["time"], data[f"xi{i}"], color=ClrMap[i-8])
    ax3.set_xlabel("$t_k\\ [s]$")
    ax3.set_ylabel("$z\\ [m]$", rotation=0)
    ax3.legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])

def robot_XY(data):
    fig = plt.figure(layout='constrained', dpi=200)
    fig.set_size_inches(5.0, 4.5)

    ax1 = fig.add_subplot()
    ax1.set_title("Formation Center")
    for i in range(4):
        ax1.plot(data[f"x{i}"], data[f"y{i}"], "o", color=ClrMap[i])
    ax1.set_xlabel("$X$")
    ax1.set_ylabel("$Y$", rotation=0)
    ax1.set(xlim=[-2.5, 2.5], ylim=[-2.5, 2.5])
    ax1.legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])

def main():
    data = get_data()

    consensus_plot(data)
    
    plt.show()

if __name__ == "__main__":
    main()
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

ClrMap  = plt.get_cmap('Set1').colors

DbFilename  = "/home/patrick/uva/data/ConsensusProject.db"
DbTable     = "DebugTable"
DbFormation = "line"


def plot_time_series(data, header, plot_label):
    fig = plt.figure(layout='constrained', dpi=200)
    fig.set_size_inches(5.5, 4.5)
    gs = GridSpec(3, 1, figure=fig)

    ax    = []
    lbls  = ["x", "y", "z"]
    ypad  = [10, 10, 10]
    xdata = data["time"]
    for i in range(3):
        ax.append(fig.add_subplot(gs[i]))

        min_y = np.inf
        max_y = -np.inf
        for j in range(4):
            ydata = data[f"{header}{4*i + j}"]
            min_y = min(min_y, np.min(ydata))
            max_y = max(max_y, np.max(ydata))
            ax[i].plot(xdata, ydata, color=ClrMap[j])
        dy = 0.05*abs(max_y - min_y)

        ax[i].set_xlabel("$t_k\\ [s]$")
        ax[i].set_ylabel(f"${plot_label}_{lbls[i]}\\ [m]$", rotation=0, labelpad=ypad[i])
        ax[i].axis([0.0, xdata[-1], min_y - dy, max_y + dy])
        ax[i].legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])

    return ax

def position_plot(data, header, labels, fig_title, ypad  = [0, 0, 0]):
    fig = plt.figure(layout='constrained', dpi=200)
    fig.set_size_inches(5.5, 4.5)
    fig.suptitle(fig_title)

    gs    = GridSpec(3, 1, figure=fig)
    tdata = data["time"]
    ax    = []

    for i in range(3):
        ax.append(fig.add_subplot(gs[i]))

        min_y = np.inf
        max_y = -np.inf
        for j in range(4):
            ydata = data[f"{header[i]}{j}"]
            min_y = min(min_y, np.min(ydata))
            max_y = max(max_y, np.max(ydata))
            ax[i].plot(tdata, ydata, color=ClrMap[j])
        dy = 0.05*abs(max_y - min_y)

        ax[i].set_xlabel("$t_k\\ [s]$")
        ax[i].set_ylabel(f"${labels[i]}\\ [m]$", rotation=0, labelpad=ypad[i])
        ax[i].legend(["$robot_1$", "$robot_2$","$robot_3$","$robot_4$"])
        if dy > 0.0:
            ax[i].axis([0.0, tdata[-1], min_y - dy, max_y + dy])

    return ax

def robot_XY(data):
    fig = plt.figure(layout='constrained', dpi=200)
    fig.set_size_inches(5.0, 4.5)

    ax1 = fig.add_subplot()
    ax1.set_title("Formation Center")
    for i in range(4):
        x = data[f"x{i}"]
        y = data[f"y{i}"]

        ax1.plot(x, y , "--", color=ClrMap[i], label=f"$robot_{i+1}$")
        ax1.plot(x[0], y[0], "o", color=ClrMap[i])
        ax1.plot(x[-1], y[-1], "s", color=ClrMap[i])
    
    ax1.set_xlabel("$X$")
    ax1.set_ylabel("$Y$", rotation=0)
    ax1.set(xlim=[-2.5, 2.5], ylim=[-2.5, 2.5])
    ax1.legend()


def robot_XYZ(data):
    fig = plt.figure(layout='constrained', dpi=200)
    fig.set_size_inches(5.0, 4.5)

    ax1 = fig.add_subplot(projection="3d")
    ax1.set_title("Formation Center")
    for i in range(4):
        x = data[f"x{i}"]
        y = data[f"y{i}"]
        z = data[f"z{i}"]
        
        ax1.plot(x, y, z, "--", color=ClrMap[i], label=f"$robot_{i+1}$")
        ax1.plot(x[0], y[0], z[0], "o", color=ClrMap[i])
        ax1.plot(x[-1], y[-1], z[-1], "s", color=ClrMap[i])
    
    ax1.set_xlabel("$X$")
    ax1.set_ylabel("$Y$", rotation=0)
    ax1.set(xlim=[-2.5, 2.5], ylim=[-2.5, 2.5])
    ax1.legend()

def main():
    data = read_all_formation_data(DbFilename, DbTable, DbFormation)

    robot_XYZ(data)
    # position_plot(data, ["Fx", "Fy", "Fz"], ["F_x", "F_y", "F_z"], "Formation Center", ypad=[15, 15, 15])
    
    plt.show()

if __name__ == "__main__":
    main()
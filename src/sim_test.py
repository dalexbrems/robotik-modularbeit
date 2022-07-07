# -*- coding: utf-8 -*-
"""
Created on Sat Jul 02

@author: Daniel Brems
"""

import robolib
import numpy as np
import plot_data
import matplotlib.pyplot as plt
from datetime import datetime


alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
a = [0.00000, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
dh_table = np.array([alpha, a, d])

q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
joint_direction = [1, 1, -1, 1, 1, 1]

robot = robolib.Robot(dh_table, q_home_offset, joint_direction)
plot = plot_data.Plot()

A = np.array([45, -65, 45, 10, 10, 0]) * np.pi/180
B = np.array([45, -65, 75, 10, 10, 0]) * np.pi/180

traj = robot.sim_movej(A, B, t=4)
#traj = robot.sim_movej(A, B)
robot.traj_to_csv_movej(traj, 'test.csv')

PA = np.array([-0.134, -0.666, 0.574, 1.294, -1.653, 2.281])
PB = np.array([-0.134, -0.666, 0.374, 1.294, -1.653, 2.281])


# TCP offset was read from URSim
# traj = robot.sim_movel(PA, PB, tcp_offset=[0.0725, -0.0731, 0.0838, 0, 0, 0])

plot.plot_all('test.csv', cols=(plot.q, plot.qd, plot.qdd), show_degrees=True)

plt.show()

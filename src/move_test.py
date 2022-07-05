# -*- coding: utf-8 -*-
"""
Created on Tue May 24 10:43:00 2022

@author: Daniel Brems
"""

import robolib
import numpy as np
import plot_data
from datetime import datetime

alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
a = [0.00000, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
dh_table = np.array([alpha, a, d])

q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
joint_direction = [1, 1, -1, 1, 1, 1]


robot = robolib.Robot(dh_table, q_home_offset, joint_direction)
plot = plot_data.Plot()


profile = '2_elbow_30_t_4'
t = datetime.now().strftime('%H_%M_%S')


# 2. Movej mit Zeitvorgabe
# A = [45; -65; 45; 10; 10; 0]
# B = [45; -65; 75; 10; 10; 0]

A = np.array([45, -65, 45, 10, 10, 0]) /180 * np.pi
#robot.movej(A)

B = np.array([45, -65, 75, 10, 10, 0]) /180 * np.pi
robot.log_data_with_movej(filename='{}_{}.csv'.format(profile, t), pose=B, t=4)
#robot.movej(B, t=4)

# 4. Movel um RZ = 0.7
# A = [45; -75; 55; -10; 10; -10]
# A Pose = [-0,134; -0,666; 0,574; 1,294; -1,653; 2,281]
# B Pose = [-0,134; -0,666; 0,574; 1,294; -1,653; 1,581]

# entweder Winkel oder Pose als Start vorgeben
# A = np.array([45, -75, 55, -10, 10, -10]) /180 * np.pi
A = np.array([-0.134, -0.666, 0.574, 1.294, -1.653, 2.281])
#robot.movej(A, isPose=True)

B = [-0.134, -0.666, 0.574, 1.294, -1.653, 1.581]
#robot.movel(B, isPose=True)

#robot.log_data_with_movel(filename='{}_{}.csv'.format(profile, t), pose=B, t=4)


# qpose = np.array([-0.13025015495206102,-0.6675856591370479,0.5122340850735787,-1.2,-2,295,2.459])
# robot.movel(qpose, isPose=True)
# ql_neu = np.array([0.18195763091493833,-0.4445283600648159,0.776892383143956,1.918867851912849,-0.4258601361137225,1.1698952970341218])
# robot.movel(ql_neu, isPose=True)

# not deeded if log_data_with_cmd works
# robot.log_data('{}/{}_{}.csv'.format(name, profile, t))

plot.add_headers('{}_{}.csv'.format(profile, t))
plot.reset_time('{}_{}_named.csv'.format(profile, t))
plot.plot_all('{}_{}_named_zeroed.csv'.format(profile, t))




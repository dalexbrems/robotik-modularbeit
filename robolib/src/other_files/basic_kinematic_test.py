import robolib
import numpy as np


alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
a = [0.00000, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
dh_table = np.array([alpha, a, d])

q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
joint_direction = [1, 1, -1, 1, 1, 1]
robot = robolib.Robot(dh_table, q_home_offset, joint_direction)
# print(robot)

ur_sim_rotvec_pose = [0.04887, -0.15926,  0.37396, 1.619, -0.868, 0.122]

T = robot.rotvec_2_T(ur_sim_rotvec_pose)
# print(T)

rotv = robot.T_2_rotvec(T)
print(rotv)

# rpy = robot.T_2_rpy(T)
# print(rpy)
#
# T = robot.rpy_2_T(rpy)
# print(T)
#
# rpy = robot.T_2_rpy(T)
# print(rpy)



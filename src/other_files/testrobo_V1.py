import robolib
import numpy as np

# T = robolib.rotvec_2_T([1, 2, 3, 1, 2, 3])
# print(T)
#
# rotvec = robolib.T_2_rotvec(T)
# print(rotvec)
#
# T1 = robolib.rotvec_2_T(rotvec)

####### DH Forware Kinematic Test #######
# # values from UR-Sim
# # /home/ur/ursim-current/.urcontrol/urcontrol.conf
# alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
# a = [0.00000, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
# d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
# q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
# joint_direction = [1, 1, -1, 1, 1, 1]
#
# ur3 = np.array([alpha, a, d, q_home_offset]).T
# # print(ur3)
#
# #q = np.array([10, -20, 30, -40, 50, -60]) / 180 * np.pi
# q = np.array([10, -30, -100, -120, 30, 60]) / 180 * np.pi
#
# T_0_6 = robolib.fk_ur_dh(ur3, q)
# print(T_0_6)
# print(robolib.T_2_rotvec(T_0_6))
# print(robolib.T_2_rpy(T_0_6))

####### Backward Kinematic ########

alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
a = [0.00000, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
dh_table = np.array([alpha, a, d])

q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
joint_direction = [1, 1, -1, 1, 1, 1]

robot = robolib.Robot(dh_table, q_home_offset, joint_direction)
print(robot)
given_tcp = [51, -170, 420, 0.6, -2.7, 1.6]
sol_bk = robot.bk(given_tcp)

# q = np.array([10, -30, -100, -120, 30, 60]) / 180 * np.pi
# T = robot.fk_ur_dh(q)
# print(T)

# ur3 = np.array([alpha, a, d, q_home_offset]).T
# T_0_6 = robolib_old.fk_ur_dh(ur3, q)
# print(T_0_6)

import robolib
import numpy as np
import time

alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
a = [0.00000, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
dh_table = np.array([alpha, a, d])

q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
joint_direction = [1, 1, -1, 1, 1, 1]
robot = robolib.Robot(dh_table, q_home_offset, joint_direction)

q0 = [0, -90, 0, -90, 0, 0]
q1 = [10, -30, -100, -120, 30, 60]

while True:
    robot.movej(q0, q_in_deg=True, t=4)
    time.sleep(5)
    robot.movej(q1, q_in_deg=True, t=4)
    time.sleep(5)
# robot.receive_data()

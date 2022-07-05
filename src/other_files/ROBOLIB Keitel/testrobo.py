# -*- coding: utf-8 -*-
"""

"""


import robolib as robolib
import numpy as np


import matplotlib.pyplot as plt

# values from UR-Sim
# /home/ur/ursim-current/.urcontrol/urcontrol.conf
a = [0.00000, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
joint_direction = [1, 1, -1, 1, 1, 1]

# TCP-Offset Konfiguration (eingestellter Greifer) von unserer Gruppe 
# [x, y, z, rx, ry, rz]
tcp_offset = np.array([0.02477, 0.003, 0.19922, 0., 0., 0.])

# dh_parameter:
dh_para = np.array([alpha, a, d, q_home_offset]).T
print("UR3-Parameterdaten:")
print(dh_para)
print("")
print("===============================")




# -----------------

# A1, 
# Aufgabe a1
# Vorgabe der Gelenkwinkel und Umrechnung in rad
q_start = np.array([5, -25, -90, -120, 45, 30]) *np.pi/180    # Aufgabe a1 q-Start
q_ende = np.array([5, -25, -65, -120, 45, 30]) *np.pi/180    # Aufgabe a1 q-Ende

trj1 = robolib.moveJ(q_start, q_ende)

# Gelenk Geschwindigkeiten
fig1 = plt.figure()

ax = fig1.add_axes([0.1, 0.1, 0.8, 0.8]) # main axes
# qd3 bewegte Achse
print(trj1)
ax.plot(trj1[2][:,0], trj1[2][:,2], label="qd3")
ax.plot(trj1[2][:,0], trj1[2][:,3], "-.k", label="qdd3")

ax.set_xlabel("Zeit in s")
ax.set_ylabel("Geschwindigkeit rad/s")
ax.set_title("Einzelachsbewegung um 25°\nGelenkgeschwindigkeit 3. Achse")
#plt.ylim(-2, 2)
plt.legend()    
# plt.show()




# ...





# A4, 
# ideale Pose aus Roboter -> sollte -2.57°, -95.44°, -101.37°, -36.43°, 25.35°, 200.36° ergeben:
poseA = np.array([0.3724 , -0.37398, 0.27822, 1.79 , 0.711, -0.197])
poseA = np.array([0.37058 , -0.37625, 0.27905, 1.789 , 0.706, -0.202])
# --> conf=6 : [  -2.57  -95.44  -101.35  -36.48   25.35  200.37 ]
poseB = np.array([0.3724 , -0.37398, 0.27822, 1.79 , 0.711, -0.197+0.5])
poseB = np.array([0.37058 , -0.37625, 0.27905, 1.789 , 0.706, -0.202 + 0.5])

trj4 = robolib.moveL(poseA, poseB)

# TCP-Geschwindigkeiten rz Ist
fig4 = plt.figure()

ax = fig4.add_axes([0.1, 0.1, 0.8, 0.8]) # main axes
#ax.plot(timearray, tcpv1_array, label="TCP-vx")
#ax.plot(timearray, tcpv2_array, label="TCP-vy")
ax.plot(trj4[:,0], trj4[:,-1], label="TCP-v rz")

ax.set_xlabel("Zeit in s")
ax.set_ylabel("Geschwindigkeit rz")
ax.set_title("moveL von A nach B\nTCP-rz-Geschwindigkeiten")
#plt.ylim(0, 0.6)
plt.legend()    
plt.show()

# --------------------
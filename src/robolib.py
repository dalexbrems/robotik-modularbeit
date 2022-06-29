import numpy as np

from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import socket
import struct
import csv
import time
# import pandas as pd


np.set_printoptions(suppress=True)


class Robot():
    def __init__(self, dh_table, q_home_offset, joint_direction, dh_classic=True):
        self.dh_table = dh_table
        self.q_home_offset = q_home_offset
        self.joint_direction = joint_direction
        self.dh_classic = dh_classic

        self.alpha = self.dh_table[0]
        self.a = self.dh_table[1]
        self.d = self.dh_table[2]

        self.HOST = "10.27.200.130"  # Robot IP
        self.TX_PORT = 30002
        self.RX_PORT = 30013

        socket.setdefaulttimeout(5.0)
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.rec_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            self.rec_sock.connect((self.HOST, self.RX_PORT))
            self.send_sock.connect((self.HOST, self.TX_PORT))
        except BlockingIOError:
            print('Socket connection could not be established')
        # self.theta = self.dh_table[3]

    def __repr__(self):
        retstring = 'This is a Representation of an 6 Axis UR3 Robot\n' \
                    'DH Parameters\nalpha {}\na {}\nd {}\n '.format(self.alpha, self.a, self.d)
        return retstring

    def rotx(self, a):
        """
        3D Rotation about x axis
        """
        ca = np.cos(a)
        sa = np.sin(a)
        T = np.array([(1, 0, 0, 0), (0, ca, -sa, 0), (0, sa, ca, 0), (0, 0, 0, 1)])
        return T

    def roty(self, a):
        """
        3D Rotation about y axis
        """
        ca = np.cos(a)
        sa = np.sin(a)
        T = np.array([(ca, 0, sa, 0), (0, 1, 0, 0), (-sa, 0, ca, 0), (0, 0, 0, 1)])
        return T

    def rotz(self, a):
        """
        3D Rotation about z axis
        """
        ca = np.cos(a)
        sa = np.sin(a)
        T = np.array([(ca, -sa, 0, 0), (sa, ca, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)])
        return T

    def transl(self, x, y, z):
        """
        Translation about x,y,z
        """
        # T = np.array([(1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (x, y, z, 1)])     # false!
        T = np.array([(1, 0, 0, x), (0, 1, 0, y), (0, 0, 1, z), (0, 0, 0, 1)])
        return T

    def Tinv(self, T):
        """
        Inverse of homogeneous trafo matrix
        """
        R = T[0:3, 0:3]  # dehomogenize
        Ri = R.transpose()
        Ti = np.eye(4)
        Ti[0:3, 0:3] = Ri
        # Ti[0:3, 3:4] = -(Ri.dot(T[0:3, 3:4]))
        Ti[0:3, 3:4] = -(Ri @ T[0:3, 3:4])  # @ operator: Matrix mulitplication; inverts the translation vector?
        return Ti

    def T_2_rotvec(self, T):
        """
        Assignment
        homogeneous trafo matrix to rotation vector representation
        The resulting vector is normalized to the rotation angle theta (in radians)
        """
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]

        diags = [T[i, i] for i in range(3)]
        s = ((sum(diags) - 1) / 2)
        theta = np.arccos(s)  # theta in radians

        if (T[2, 1] - T[1, 2]) < 0:
            theta = theta - 2 * np.pi
        # print(theta)

        help_vec = np.array([T[2, 1] - T[1, 2], T[0, 2] - T[2, 0], T[1, 0] - T[0, 1]])
        rot_vec_k = (1 / (2 * np.sin(theta))) * help_vec  # rotation vector depicted in k values (w/out theta)
        # print(rot_vec_k)

        rot_vec_normalized = np.dot(rot_vec_k, theta)  # normalize the vector to theta (in radians)
        # rot_vec_normalized = rot_vec_k *  theta  # normalize the vector to theta (in radians)
        pose = np.array((x, y, z, *rot_vec_normalized))
        return pose

    def rotvec_2_T(self, pose):
        """
        Assignment
        pose in the form of xyzrxryrz with rotation vector representation to homogeneous trafo matrix
        """

        th = np.sqrt(pose[3] ** 2 + pose[4] ** 2 + pose[5] ** 2)
        if th:
            kx = pose[3] / th
            ky = pose[4] / th
            kz = pose[5] / th
        else:
            kx = 0
            ky = 0
            kz = 0

        # print('Theta Radians:', th)
        # print('Theta Degrees:', np.degrees(th))
        # print('K Vector: ', kx, ky, kz)

        v = 1 - np.cos(th)
        s = np.sin(th)
        c = np.cos(th)

        r11 = kx * kx * v + c
        r12 = kx * ky * v - kz * s
        r13 = kx * kz * v + ky * s

        r21 = kx * ky * v + kz * s
        r22 = ky * ky * v + c
        r23 = ky * kz * v - kx * s

        r31 = kx * kz * v - ky * s
        r32 = ky * kz * v + kx * s
        r33 = kz * kz * v + c

        T = np.array([[r11, r12, r13, pose[0]],
                      [r21, r22, r23, pose[1]],
                      [r31, r32, r33, pose[2]],
                      [0, 0, 0, 1]])

        return T

    def T_2_rpy(self, T):
        """
        Assignment
        homogeneous trafo matrix to pose with roll-pitch-yaw x,y,z,r,p,y (Euler Angles)
        TODO: define order of output angles (in USim it's gamma, beta, alpha)
        """
        x = T[0, 3]
        y = T[1, 3]
        z = T[2, 3]

        # print(x, y, z)

        r_11 = T[0, 0]
        r_21 = T[1, 0]
        r_31 = T[2, 0]
        r_32 = T[2, 1]
        r_33 = T[2, 2]

        beta = np.arctan2(-r_31, np.sqrt(r_11 ** 2 + r_21 ** 2))
        cb = np.cos(beta)
        alpha = np.arctan2(r_21 / cb, r_11 / cb)
        gamma = np.arctan2(r_32 / cb, r_33 / cb)

        # euler_angles = np.degrees((alpha, beta, gamma))
        # pose = np.array((x, y, z, *euler_angles))
        pose = np.array((x, y, z, alpha, beta, gamma))

        return pose

    def rpy_2_T(self, pose):
        """
        Assignment
        pose with roll-pitch-yaw to homogeneous trafo matrix. Order of axis rotations: z, y, x
        TODO: Passt noch nicht!
        """
        x = pose[0]
        y = pose[1]
        z = pose[2]

        # alpha = np.radians(pose[3])  # rotation about z-axis
        # beta = np.radians(pose[4])  # rotation about y-axis
        # gamma = np.radians(pose[5])  # rotation about x-axis

        alpha = pose[3]  # rotation about z-axis
        beta = pose[4]  # rotation about y-axis
        gamma = pose[5]  # rotation about x-axis

        ca = np.cos(alpha)
        sa = np.sin(alpha)

        cb = np.cos(beta)
        sb = np.sin(beta)

        cg = np.cos(gamma)
        sg = np.sin(gamma)

        r11 = ca * cb
        r12 = (ca * sb * sg) - (sa * cg)
        r13 = (ca * sb * cg) + (sa * sg)

        r21 = sa * cb
        r22 = (sa * sb * sg) + (ca * cg)
        r23 = (sa * sb * cg) - (ca * sg)

        r31 = -sb
        r32 = cb * sg
        r33 = cb * cg

        T = np.array([[r11, r12, r13, x],
                      [r21, r22, r23, y],
                      [r31, r32, r33, z],
                      [0, 0, 0, 1]])
        return T

    def dh(self, alpha, a, d, theta):
        """
        Denavit-Hartenberg (classic)
        """

        ca = np.cos(alpha)
        sa = np.sin(alpha)
        ct = np.cos(theta)
        st = np.sin(theta)

        # TODO: change comparison of irrational numbers in if statement
        # r00 = 0 if theta == np.pi / 2 or theta == -np.pi / 2 else ct
        # r01 = 0 if alpha == np.pi / 2 or alpha == -np.pi / 2 else -st * ca
        # r02 = st * sa
        # r03 = 0 if theta == np.pi / 2 or theta == -np.pi / 2 else a * ct
        #
        # r10 = st
        # r11 = 0 if theta == np.pi / 2 or alpha == np.pi / 2 or theta == -np.pi / 2 or alpha == -np.pi / 2 else ct * ca
        # r12 = 0 if theta == np.pi / 2 or theta == -np.pi / 2 else -ct * sa
        # r13 = a * st
        #
        # r21 = sa
        # r22 = 0 if alpha == np.pi / 2 or alpha == -np.pi / 2 else ca

        r00 = ct
        r01 = -st * ca
        r02 = st * sa
        r03 = a * ct

        r10 = st
        r11 = ct * ca
        r12 = -ct * sa
        r13 = a * st

        r21 = sa
        r22 = ca

        T = np.array([[r00, r01, r02, r03],
                      [r10, r11, r12, r13],
                      [0, r21, r22, d],
                      [0, 0, 0, 1]])

        return T

    def dhm(self, alpha, a, d, theta):
        """
        Denavit-Hartenberg (modified)
        """

        ca = np.cos(alpha)
        sa = np.sin(alpha)
        ct = np.cos(theta)
        st = np.sin(theta)

        r00 = ct
        r01 = -st

        r10 = st * ca
        r11 = ct * ca
        r12 = -sa
        r13 = -d * sa

        r20 = st * sa
        r21 = ct * sa
        r22 = ca
        r23 = d * ca

        T = np.array([[r00, r01, 0, a],
                      [r10, r11, r12, r13],
                      [r20, r21, r22, r23],
                      [0, 0, 0, 1]])

        return T

    def fk_ur_dh(self, q):
        """
        Forward Kinematics for UR type robots
        alpha = 0, a = 1, d = 2, theta = 3
        """

        dh_params = np.array([self.alpha, self.a, self.d, self.q_home_offset]).T
        T_0_6 = np.eye(4)

        for param, qn in zip(dh_params, q):
            T_0_6 = T_0_6 @ self.dh(param[0], param[1], param[2], qn)

        # for i in range(dh_para):
        #     T_0_6 = T_0_6 @ dh(dh_para[i][0], dh_para[i][1], dh_para[i][2], q[i])
        return T_0_6

    def ik_ur(self, pose):
        """Computes the Motor angles, given a pose with rotation vector"""
        x = pose[0]
        y = pose[1]
        z = pose[2]

        rx = pose[3]
        ry = pose[4]
        rz = pose[5]


        # q1 (Carousel angle)
        T_0_6 = self.rotvec_2_T(pose)               # Trafo Matrix for given TCP

        # coordinates of the hand axis intersection point in System 6
        s_6 = np.array([0, 0, -self.d[5], 1])
        s_0 = np.dot(T_0_6, s_6)

        alpha_1 = np.arctan2(s_0[1], s_0[0])
        R = np.sqrt(s_0[0]**2 + s_0[1]**2)
        alpha_2 = np.arccos(self.d[3] / R)

        q1 = alpha_1 + alpha_2 + (np.pi/2)

        # q5
        s1 = np.sin(q1)
        c1 = np.cos(q1)

        q5 = np.arccos((x*s1 - y*c1 - self.d[3]) / self.d[5])

        # q6

        s5 = np.sin(q5)
        c5 = np.cos(q5)

        term_1 = (-T_0_6[0, 1]*s1 + T_0_6[1, 1]*c1) / s5
        term_2 = (-T_0_6[0, 0]*s1 + T_0_6[1, 0]*c1) / s5
        q6 = np.arctan2(term_1, term_2)

        # q2, q3, q4 ergeben sich aus ebenem Problem mit drei parallelen Achsen

    def rk_ur(self, xyzrxryrz: np.array) -> np.array:

        """
        :param xyzrxryrz: pose
        """
        sols = []

        x = xyzrxryrz[0]
        y = xyzrxryrz[1]
        z = xyzrxryrz[2]
        rx = xyzrxryrz[3]
        ry = xyzrxryrz[4]
        rz = xyzrxryrz[5]

        # alpha = dh_paras[:, 0]
        # a = dh_paras[:, 1]
        # d = dh_paras[:, 2]

        alpha = self.alpha
        a = self.a
        d = self.d
        print('D ist:', d)

        l1 = d[0]
        l4 = d[3]
        l5 = d[4]
        l6 = d[5]

        T_06 = self.rotvec_2_T(xyzrxryrz)
        O_6_5 = np.array([0, 0, -l6, 1])
        O_0_5 = T_06 @ O_6_5
        r11 = T_06[0, 0]
        r12 = T_06[0, 1]
        r21 = T_06[1, 0]
        r22 = T_06[1, 1]

        # q1 (Carousel angle)
        O_0_5_x = O_0_5[0]
        O_0_5_y = O_0_5[1]
        alpha_1 = np.arctan2(O_0_5_y, O_0_5_x)
        R = np.sqrt(O_0_5_x ** 2 + O_0_5_y ** 2)
        alpha_2 = np.arccos(l4 / R)

        q1_lst = np.array([alpha_1 + alpha_2 + np.pi / 2,
                           alpha_1 - alpha_2 + np.pi / 2])

        for i, q1 in enumerate(q1_lst):
            s1 = np.sin(q1)
            c1 = np.cos(q1)
            q5_lst = np.array([np.arccos((x * s1 - y * c1 - l4) / l6),
                               -np.arccos((x * s1 - y * c1 - l4) / l6)])
            for j, q5 in enumerate(q5_lst):
                s5 = np.sin(q5)
                c5 = np.cos(q5)
                q6_lst = [np.arctan2((-r12 * s1 + r22 * c1) / s5, (r11 * s1 - r21 * c1) / s5)]
                for k, q6 in enumerate(q6_lst):
                    q156 = np.array([q1, 0, 0, 0, q5, q6])
                    ur3 = np.array([alpha, a, d, q156]).T
                    a2 = ur3[1, 1]
                    a3 = ur3[2, 1]
                    T_10 = self.Tinv(self.dh(ur3[0, 0], ur3[0, 1], ur3[0, 2], ur3[0, 3]))
                    T_65 = self.Tinv(self.dh(ur3[5, 0], ur3[5, 1], ur3[5, 2], ur3[5, 3]))
                    T_54 = self.Tinv(self.dh(ur3[4, 0], ur3[4, 1], ur3[4, 2], ur3[4, 3]))
                    T_14 = T_10 @ T_06 @ T_65 @ T_54
                    P_14x = T_14[0, 3]
                    P_14y = T_14[1, 3]
                    P0_14 = np.sqrt(T_14[0, 3] ** 2 + T_14[1, 3] ** 2)
                    q3_lst = np.array([np.arccos((P0_14 ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3)),
                                       -np.arccos((P0_14 ** 2 - a2 ** 2 - a3 ** 2) / (2 * a2 * a3))])
                    for l, q3 in enumerate(q3_lst):
                        P0_14 = np.sqrt(T_14[0, 3] ** 2 + T_14[1, 3] ** 2)
                        q2_lst = np.array([np.arctan2(-P_14y, -P_14x) - np.arcsin(-a3 * np.sin(q3) / P0_14)])
                        for m, q2 in enumerate(q2_lst):
                            ur3[1, 3] = q2
                            ur3[2, 3] = q3
                            T_32 = self.Tinv(self.dh(ur3[2, 0], ur3[2, 1], ur3[2, 2], ur3[2, 3]))
                            T_21 = self.Tinv(self.dh(ur3[1, 0], ur3[1, 1], ur3[1, 2], ur3[1, 3]))
                            T_34 = T_32 @ T_21 @ T_14
                            T_34_x = T_34[0, 0]
                            T_34_y = T_34[1, 0]
                            q4_lst = [np.arctan2(T_34_y, T_34_x)]
                            for n, q4 in enumerate(q4_lst):
                                # print(np.rad2deg(q1), np.rad2deg(q2), np.rad2deg(q3), np.rad2deg(q4), np.rad2deg(q5), np.rad2deg(q6))
                                sols.append(np.array([q1, q2, q3, q4, q5, q6]))
        return np.array(sols)

    def jacobian_matrix(self, q: np.ndarray, round=False) -> np.array:

        jacobian = np.zeros((6, 6))

        T_0_6 = self.fk_ur_dh(q)
        point_end = T_0_6[0:3, 3]

        T_0_i = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])

        for i in range(6):

            if i == 0:
                T_0_i = T_0_i
            else:
                T = self.dh(self.alpha[i - 1], self.a[i - 1], self.d[i - 1], q[i - 1])
                T_0_i = np.dot(T_0_i, T)

            z_i = T_0_i[0:3, 2]
            p_i = T_0_i[0:3, 3]
            r = point_end - p_i
            jacobian[0:3, i] = np.cross(z_i, r)
            jacobian[3:6, i] = z_i

            if round:
                jacobian[0:3, i] = np.round(np.cross(z_i, r), 3)
                jacobian[3:6, i] = np.round(z_i, 3)

        return jacobian

    def movej(self, q, q_in_deg=False, isPose=False, a=1.4, v=1.05, t=0, r=0):
        if isPose:
            command = 'movej(p[{}, {}, {}, {}, {}, {}],' \
                      ' a={}, v={}, t={}, r={})\n'.format(*q, a, v, t, r)
            
        elif q_in_deg:
            command = 'movej([d2r({}), d2r({}), d2r({}), d2r({}), d2r({}), d2r({})],' \
                      ' a={}, v={}, t={}, r={})\n'.format(*q, a, v, t, r)
        else:
            command = 'movej([{}, {}, {}, {}, {}, {}],' \
                      ' a={}, v={}, t={}, r={})\n'.format(*q, a, v, t, r)

        self.send_sock.send(command.encode('ascii'))

    def movel(self, q, q_in_deg=False, isPose=False, a=1.4, v=1.05, t=0, r=0):
        if isPose:
            command = 'movel(p[{}, {}, {}, {}, {}, {}],' \
                      ' a={}, v={}, t={}, r={})\n'.format(*q, a, v, t, r)
            print(command)
            
        elif q_in_deg:
            command = 'movel([d2r({}), d2r({}), d2r({}), d2r({}), d2r({}), d2r({})],' \
                      ' a={}, v={}, t={}, r={})\n'.format(*q, a, v, t, r)
        else:
            command = 'movel([{}, {}, {}, {}, {}, {}],' \
                      ' a={}, v={}, t={}, r={})\n'.format(*q, a, v, t, r)

        self.send_sock.send(command.encode('ascii'))

    def receive_data(self):
        r = self.rec_sock.recv(1116)  # 8 ms

        sb = 0  # Start byte
        a = struct.unpack('>i', r[sb:sb + 4])[0]
        # print('Message Size: ', a)

        sb = 4  # Start byte
        t = struct.unpack('!d', r[sb:sb + 8])[0]
        # print('Time: ', t)

        sb = 12  # Start byte
        dt = struct.unpack('!dddddd ', r[sb:sb + 8 * 6])
        qt = np.array(dt)
        
        sb = 60  # Start byte
        ddt = struct.unpack('!dddddd ', r[sb:sb + 8 * 6])
        qdt = np.array(ddt)
        
        sb = 108  # Start byte
        dddt = struct.unpack('!dddddd ', r[sb:sb + 8 * 6])
        qddt = np.array(dddt)

        sb = 252  # Start byte
        d = struct.unpack('!dddddd ', r[sb:sb + 8 * 6])
        qn = np.array(d)
        # print(qn)
        # print('Joints: ', qn / np.pi * 180)

        sb = 300
        sp = struct.unpack('!dddddd', r[sb:sb + 8 * 6])
        q_speed = np.array(sp)

        sb = 444  # Start byte
        d = struct.unpack('!dddddd', r[sb:sb + 8 * 6])
        pose = np.array(d)
        # print('Pose', pose)
        
        sb = 492
        sp = struct.unpack('!dddddd', r[sb:sb + 8 * 6])
        tcp_speed = np.array(sp) 
    
        
        return {'Message Size': a, 'Time': t, 'Joints': qn, 'QSpeeds': q_speed, 'Pose': pose, 'TCPSpeed': tcp_speed, 'Joints_t': qt, 'Speed_t': qdt, 'Accel_t': qddt}

    def log_data(self, filename, duration=7):
        t_begin = time.time()
        #cnt = 0

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)

            while time.time() <= t_begin + duration:
                r = self.receive_data()
                writer.writerow([r['Time'], *r['Joints'], *r['QSpeeds'], *r['Pose'], *r['TCPSpeed'], *r['Joints_t'], *r['Speed_t'], *r['Accel_t']])
                # writer.writerow([r['Time'], r['Joints'], r['Pose'], r['Speed']])

                # writer.writerow([r['Time']])
#                cnt += 1
#                if r['QSpeeds'].all() == 0 and time.time() >= t_begin + 1:
#                    print('No speed detected, timestamp:', time.time())
#                    break


def bk_test():
    """
    Backward Kinematics
    """
    # gesucht: f(x) = 2 mit f(x) = sin(x) + x^3 + 3*x^2

    x = np.arange(-4, 2, 0.01)

    def f(x):
        return np.sin(x) + x ** 3 + 3 * x ** 2

    y = f(x)

    plt.plot(x, y)
    plt.ylim((-4, 4))
    plt.grid()

    def func(x):
        return f(x) - 2

    x0 = -1
    sol1 = fsolve(func, x0)
    print(sol1)
    plt.plot(sol1, f(sol1), 'ro')

    x0 = -3
    sol2 = fsolve(func, x0)
    print(sol2)
    plt.plot(sol2, f(sol2), 'ro')

    x0 = 5
    sol3 = fsolve(func, x0)
    print(sol3)
    plt.plot(sol3, f(sol3), 'ro')

    plt.show()
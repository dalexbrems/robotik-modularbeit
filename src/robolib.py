import numpy as np

# from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import socket
import struct
import csv
import time
import pandas as pd


np.set_printoptions(suppress=True)

T_MAX = 30  # maximum duration for a robot movement

class Robot:
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

        # try:
        #     self.rec_sock.connect((self.HOST, self.RX_PORT))
        #     self.send_sock.connect((self.HOST, self.TX_PORT))
        # except BlockingIOError:
        #     print('Socket connection could not be established')

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
        print(command)
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


    def log_data_with_movej(self, filename, pose=0, t=4, duration=7, t_lead=2):
        t_begin = time.time()
        # cnt = 0
        cmd_sent = False

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)

            while time.time() <= t_begin + duration:
                r = self.receive_data()
                writer.writerow([r['Time'], *r['Joints'], *r['QSpeeds'], *r['Pose'], *r['TCPSpeed'], *r['Joints_t'],
                                 *r['Speed_t'], *r['Accel_t']])

                # sends the command once when reaching the lead time
                if time.time() >= t_begin + t_lead and not cmd_sent:
                    print(f'Command was sent at {time.time()}.')
                    cmd_sent = True
                    self.movej(pose, isPose=False, t=t)

    def log_data_with_movel(self, filename, pose=0, t=4, duration=7, t_lead=2):
        t_begin = time.time()
        print(f'T begin: {time.localtime(t_begin)}')
        # cnt = 0
        cmd_sent = False

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)

            while time.time() <= t_begin + duration:
                r = self.receive_data()
                writer.writerow([r['Time'], *r['Joints'], *r['QSpeeds'], *r['Pose'], *r['TCPSpeed'], *r['Joints_t'],
                                 *r['Speed_t'], *r['Accel_t']])

                # sends the command once when reaching the lead time
                if time.time() >= t_begin + t_lead and not cmd_sent:
                    print(f'Command was sent at {time.localtime(time.time())}.')
                    cmd_sent = True
                    self.movel(pose, isPose=True, t=t)

    def sim_movej(self, q, q_target, a=1.4, v=1.05, t=0):
        """
        Own implementation of the ur 'movej' command.
        Move to position (linear in joint space)

        :param list q: current drive angles (rad)
        :param list q_target: target drive angles (rad)
        :param float a: joint acceleration of the leading axis (rad/s²)
        :param float v: joint speed of leading axis (rad/s)
        :param float t: time allocated for the movement (s)
        :return: trajectory data of q, qd, qdd
        :rtype: list(list(float))
        """
        a = abs(a)
        v = abs(v)
        # get the drive angle difference
        dq = q_target - q
        # print(f'dq = {dq}')

        # this ensures priority of the duration parameter t over v and a
        if t == 0:
            traj = self.trajectory(dq, v_max=v, a_max=a)
        elif t > 0 and t <= T_MAX:
            traj = self.trajectory_t(dq, a_max=1.4, t=t)
        else:
            raise ValueError(f'The specified duration ({t}s) is too long. Max. {T_MAX}s')

        # as the trajectory only computes the relative movement, add start angles to the data
        for i, ax in enumerate(traj):
            for dp in ax:
                dp[1] += q[i]

        return traj

    def sim_movel(self, pose, pose_target, tcp_offset, a=1.2, v=0.25, t=0):
        """
        Own implementation of the ur 'movel' command.
        Move the Tool Center Point linearly in space

        :param list pose: current pose [x, y, z, rx, ry, rz] (m)
        :param list pose_target: target pose [x, y, z, rx, ry, rz] (m)
        :param list tcp_offset: offset of the TCP [x, y, z, rx, ry, rz] (m)
        :param float v: TCP speed (m/s)
        :param float a: TCP acceleration (m/s²)
        :param float t: time allocated for the movement (s)
        :return: trajectory data of q, qd, qdd and TCP pose
        :rtype: list(list(float))
        """

        # remove the TCP offset from start and end pose to get the robot tcp
        T_0_TCP = self.rpy_2_T(pose)     # Trafo Matrix from base to TCP
        T_0_6 = np.dot(self.transl(tcp_offset[0], tcp_offset[1], tcp_offset[2]), T_0_TCP)

        print(f'Pose with TCP offset removed: {self.T_2_rpy(T_0_6)}\nSomehow, the Rotations dont stay the same.'
              f'Looks like a bug in T_2_rpy...')
        pose = self.T_2_rpy(T_0_6)

        T_0_TCP = self.rpy_2_T(pose_target)
        T_0_6 = np.dot(self.transl(tcp_offset[0], tcp_offset[1], tcp_offset[2]), T_0_TCP)

        print(f'Pose with TCP offset removed: {self.T_2_rpy(T_0_6)}\nSomehow, the Rotations dont stay the same.'
              f'Looks like a bug in T_2_rpy...')
        pose_target = self.T_2_rpy(T_0_6)

        # get the pose difference
        dp = pose_target - pose


        # can use the same trajectory computation as for movej
        if t == 0:
            traj = self.trajectory(dp, v_max=v, a_max=a)
        elif t > 0 and t <= T_MAX:
            traj = self.trajectory_t(dp, a_max=1.4, t=t)
        else:
            raise ValueError(f'The specified duration ({t}s) is too long. Max. {T_MAX}s')

        # as the trajectory only computes the relative movement, add start angles to the data
        for i, ax in enumerate(traj):
            for dp in ax:
                dp[1] += pose[i]

    #######  Compute Trajectories  #######
    def trajectory(self, dq, v_max, a_max):
        """
        Compute a trajectory for the given drive angles without given duration.
        First identify the leading axis and then compute switching times of all other axis.

        :param list dq: list of drive angles differences
        :param float v_max: max. velocity of the leading axis (rad/s)
        :param float a_max: max. acceleration of the leading axis (rad/s²)
        :return: trajectory data for angle differences
        :rtype: narray(narray(narray(floats)))
        """

        # identify the leading axis (slowest axis)

        t_total = 0      # total time
        la = None        # leading axis

        for ax in range(len(dq)):
            t_ax = self.traj_sp(dq[ax], v_max, a_max)[2]
            # print(f'T_ax at {ax}: {t_ax} secs.')
            if t_ax > t_total:
                t_total = t_ax  # new total time
                la = ax         # new leading axis

        sp_la = self.traj_sp(dq[la], v_max, a_max)      # switching points of the leading axis
        print(f'Leading axis (ts_1, ts_2, t_total: {sp_la}')

        # compute velocities and acceleration of other axis
        v_a = np.zeros((6, 2))
        for ax in range(len(dq)):
            t_tot = sp_la[2]
            t_s1 = sp_la[0]
            v_B = dq[ax]/(t_tot - t_s1)
            a_A = dq[ax]/(t_tot*t_s1 - t_s1**2)
            v_a[ax, 0] = v_B
            v_a[ax, 1] = a_A
            print(v_a)


        # print(f'Velocities and Acceleration of all axes: {v_a}')

        # get angle, velocity and acceleration of each axis for the given switching points
        # time-discrete for the UR frequency of 8ms
        ret_arr = []
        for i, ax in enumerate(dq):
            ret_arr.append(self.traj_data_points(ax, v_max=v_a[i, 0], a_max=v_a[i, 1], sp_la=sp_la))

        return np.array(ret_arr, dtype=object)

    def trajectory_t(self, dq, t, v_max=3, a_max=2.5):
        """
        Compute a trajectory for the given drive angles without given duration.
        First identify the leading axis and then compute switching times of all other axis.

        :param list dq: list of drive angles differences
        :param float v_max: optional: max. velocity that one axis is allowed to reach (rad/s)
        :param float a_max: optional: max. acceleration that one axis is allowed to reach(rad/s²)
        :param float t: total duration of the movement
        :return: trajectory data for angle differences
        :rtype: narray(narray(narray(floats)))
        """


        ####### 25% approach #######
        # this works as expected, but the profile observed in the lab was a triangle profile

        # compute v_B and a_A to achieve a duration of t secs
        # slide 29: 25% profile (acceleration and deceleration phase take up 25% of the complete profile each
        # t_s_1 = 0.25 * t     # first switching point is reached after 25% of the desired duration
        # t_s_2 = 0.75 * t
        #
        # sp = [t_s_1, t_s_2, t]   # switching points
        #
        # v_a = np.zeros((6, 2))
        # for ax in range(len(dq)):
        #     # slide 29
        #     v_B = (16 * dq[ax]) / (12 * t)
        #     a_A = (16 * dq[ax]) / (3 * t**2)
        #
        #     # check if velocity and acceleration are safe
        #     if v_B > v_max or a_A > a_max:
        #         raise ValueError('The defined movement duration is to short. Velocity or acceleration of one or '
        #                          'multiple axes would exceed maximum values')
        #
        #     v_a[ax, 0] = v_B
        #     v_a[ax, 1] = a_A

        # as the observed profile for a movej command with t=4 was a triangle: simple approach with switching points
        # and maximum acceleration (slide 27)
        t_s_1 = 0.5 * t     # first switching point is reached after 25% of the desired duration
        sp = [t_s_1, 0, t]   # switching points

        v_a = np.zeros((6, 2))
        for ax in range(len(dq)):
            # slide 29
            v_B = dq[ax]/(t - t_s_1)
            a_A = dq[ax]/(t * t_s_1 - t_s_1**2)
            if v_B > v_max or a_A > a_max:
                raise ValueError('The defined movement duration is to short. Velocity or acceleration of one or '
                                 'multiple axes would exceed maximum values')
            v_a[ax, 0] = v_B
            v_a[ax, 1] = a_A

        print(f'Velocities and Acceleration of all axes: {v_a}')

        # get angle, velocity and acceleration of each axis for the given switching points
        # time-discrete for the UR frequency of 8ms
        ret_arr = []
        for i, ax in enumerate(dq):
            ret_arr.append(self.traj_data_points(ax, v_max=v_a[i, 0], a_max=v_a[i, 1], sp_la=sp))

        return np.array(ret_arr, dtype=object)

    def traj_data_points(self, dq, v_max, a_max, sp_la, t_fill=1):
        """
        Returns angle, velocity and acceleration for a given single axis movement
        discretized to 8ms steps

        :param float dq: drive angle travel in radians
        :param float v_max: max. velocity of the axis (rad/s)
        :param float a_max: max. acceleration of the axis (rad/s²)
        """
        t_s_1 = sp_la[0]
        t_s_2 = sp_la[1]
        t_tot = sp_la[2]

        dt = 0.008  # timestep

        step_cnt = int(np.ceil(t_tot/dt)) + 10          # total number of steps that contain data
        fill_step_cnt = int(np.ceil(t_fill/dt))    # number of data points to fill before and after the payload data
        # print(f'Step count: {step_cnt}')

        ret_array = []

        # check if dq is approx. zero -> no movement, empty array

        # rounds dq to 6 digits
        if np.round(dq, 5) == 0:
            for s in range(step_cnt):
                t = s * dt  # current timestep in ms
                ret_array.append([t, 0, 0, 0])
            return ret_array

        print(f't_s_2 when entering traj_data_points: {t_s_2}')
        # triangle profile
        if t_s_2 == 0:
            qs = 0.5 * dq
            # ts = np.sqrt(abs(dq/a_max))
            ts = t_s_1
            vs = np.sqrt(abs(dq * a_max))

            # invert velocity for negative movements
            if dq < 0:
                vs = -vs

            # ret_array.append([-dt, 0, 0, 0])

            for s in range(step_cnt):
                t = s * dt    # current timestep in ms

                # print(f'timestep: {t}, ts_1: {t_s_1}')
                # slide 22
                if t <= t_s_1:
                    qt = 0.5 * a_max * t**2
                    vt = a_max * t
                    at = a_max
                    ret_array.append([t, qt, vt, at])

                # slide 23
                if t > t_s_1 and t <= t_tot:

                    # this adds the switching point so the transition of the triangle is correct
                    # if t <= t_s_1 + dt:
                    #     ret_array.append([ts, qs, vs, 0])

                    # square the last term to achieve smoothing
                    qt = qs + (vs * (t - ts)) - (0.5 * a_max * (t - ts) ** 2)
                    vt = vs - a_max*(t - ts)
                    at = -a_max
                    ret_array.append([t, qt, vt, at])

                if t > t_tot:
                    qt = dq
                    vt = 0
                    at = 0
                    ret_array.append([t, qt, vt, at])

        # trapezoidal profile
        else:
            qs_1 = 0.5 * (v_max**2/a_max)
            qs_2 = dq - qs_1

            for s in range(step_cnt):
                t = s * dt    # current timestep in ms

                # section A
                if t <= t_s_1:
                    qt = 0.5 * a_max * t**2
                    vt = a_max * t
                    at = a_max
                    ret_array.append([t, qt, vt, at])

                # section B
                if t > t_s_1 and t < t_s_2:
                    qt = qs_1 + v_max * (t - t_s_1)
                    vt = v_max
                    at = 0
                    ret_array.append([t, qt, vt, at])

                # section C
                if t >= t_s_2 and t <= t_tot:
                    qt = qs_2 + (v_max + ((a_max/v_max) * dq)) * (t - t_s_2) - 0.5 * a_max * (t**2 - t_s_2**2)
                    vt = v_max - a_max*(t - t_s_2)
                    at = -a_max
                    ret_array.append([t, qt, vt, at])

                if t > t_tot:
                    qt = dq
                    vt = 0
                    at = 0
                    ret_array.append([t, qt, vt, at])

        return ret_array

    def traj_sp(self, dq, v_max, a_max):
        """
        Computes the switching points for the given single axis movement

        :param float dq: drive angle travel in radians
        :param float v_max: max. velocity of the axis (rad/s)
        :param float a_max: max. acceleration of the axis (rad/s²)

        :return: (t_s_1, t_s_2, t_total)
        :rtype: tuple
        """

        # distinguish between trapeziodal and triangle profile

        q_limit = (v_max**2)/a_max
        # triangle profile
        if abs(dq) <= q_limit:
            t_s_1 = np.sqrt(abs((dq/a_max)))  # switching time
            t_s_2 = 0
            t_total = 2*t_s_1
            # print('Triangle profile')
        # trapezoidal profile
        else:
            print('Trapezoidal profile')
            t_s_1 = v_max/a_max
            t_s_2 = abs(dq/v_max)
            t_total = t_s_1 + t_s_2

        return t_s_1, t_s_2, t_total

    def traj_to_csv_movej(self, trajectories, filename):
        # Todo: nice to have

        q = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6']
        qd = ['qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6']
        # pose = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        qddt = ['qdd1', 'qdd2', 'qdd3', 'qdd4', 'qdd5', 'qdd6']

        # get time data for whole csv
        # ret = pd.DataFrame(pd.DataFrame(data=trajectories[0]).iloc[:, 0])

        ret = pd.DataFrame([item[0] for item in trajectories[0]], columns=['t'])

        print(ret)
        for i, traj in enumerate(trajectories):
            df = pd.DataFrame(data=traj, columns=['t', 'q', 'qd', 'qdd'])
            ret.insert(i+1, q[i], df.loc[:, 'q'])
            ret.insert(i+2, qd[i], df.loc[:, 'qd'])
            ret.insert(i+3, qddt[i], df.loc[:, 'qdd'])
        ret.to_csv(filename, sep=',', index=False)

    def traj_to_csv_movel(self, trajectories, filename):
        # Todo: nice to have

        q = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6']
        qd = ['qd1', 'qd2', 'qd3', 'qd4', 'qd5', 'qd6']
        # pose = ['x', 'y', 'z', 'rx', 'ry', 'rz']
        qddt = ['qdd1', 'qdd2', 'qdd3', 'qdd4', 'qdd5', 'qdd6']

        # get time data for whole csv
        # ret = pd.DataFrame(pd.DataFrame(data=trajectories[0]).iloc[:, 0])

        ret = pd.DataFrame([item[0] for item in trajectories[0]], columns=['t'])

        print(ret)
        for i, traj in enumerate(trajectories):
            df = pd.DataFrame(data=traj, columns=['t', 'q', 'qd', 'qdd'])
            ret.insert(i+1, q[i], df.loc[:, 'q'])
            ret.insert(i+2, qd[i], df.loc[:, 'qd'])
            ret.insert(i+3, qddt[i], df.loc[:, 'qdd'])
        ret.to_csv(filename, sep=',', index=False)
# def bk_test():
#     """
#     Backward Kinematics
#     """
#     # gesucht: f(x) = 2 mit f(x) = sin(x) + x^3 + 3*x^2
#
#     x = np.arange(-4, 2, 0.01)
#
#     def f(x):
#         return np.sin(x) + x ** 3 + 3 * x ** 2
#
#     y = f(x)
#
#     plt.plot(x, y)
#     plt.ylim((-4, 4))
#     plt.grid()
#
#     def func(x):
#         return f(x) - 2
#
#     x0 = -1
#     sol1 = fsolve(func, x0)
#     print(sol1)
#     plt.plot(sol1, f(sol1), 'ro')
#
#     x0 = -3
#     sol2 = fsolve(func, x0)
#     print(sol2)
#     plt.plot(sol2, f(sol2), 'ro')
#
#     x0 = 5
#     sol3 = fsolve(func, x0)
#     print(sol3)
#     plt.plot(sol3, f(sol3), 'ro')
#
#     plt.show()
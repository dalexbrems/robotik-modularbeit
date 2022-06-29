import numpy as np

def rotx(a):
    """
    Rotation about x axis
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(1, 0,    0, 0), 
                  (0, ca, -sa, 0), 
                  (0, sa,  ca, 0), 
                  (0,  0,   0, 1)])
    return T


def roty(a):
    """
    Rotation about y axis
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(ca, 0, sa, 0), (0, 1, 0, 0), (-sa, 0, ca, 0), (0, 0, 0, 1)])
    return T


def rotz(a):
    """
    Rotation about z axis
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(ca, -sa, 0, 0), (sa, ca, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)])
    return T


def transl(x, y, z):
    """
    Translation about x,y,z
    """
    T = np.array([(1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (x, y, z, 1)])
    return T


def Tinv(T):
    """
    Inverse of homogeneous trafo matrix
    """
    R = T[0:3, 0:3]
    Ri = R.transpose()
    Ti = np.eye(4)
    Ti[0:3, 0:3] = Ri
    #Ti[0:3, 3:4] = -(Ri.dot(T[0:3, 3:4]))
    Ti[0:3, 3:4] = -(Ri @ T[0:3, 3:4])
    return Ti


def T_2_rotvec(T):
    """
    homogeneous trafo matrix to rotation vector representation
    """
    pose = np.zeros((6))
    # todo
    # theta in Rad, numpy uses Rad
    xx = T[2, 1] - T[1, 2]
    yy = T[0, 2] - T[2, 0]
    zz = T[1, 0] - T[0, 1]
    r = np.sqrt(xx**2 + yy**2 + zz**2)
    tr = T[0, 0] + T[1, 1] + T[2, 2]
    th = np.arctan2(r, (tr-1))
    kx = xx/r
    ky = yy/r
    kz = zz/r

    # th = np.arccos((tr - 1)/2)
    # n  = 1/(2*np.sin(th))
    # kx = n * xx1
    # ky = n * yy
    # kz = n * zz

    # print('Theta: ', th)

    rx = kx * th
    ry = ky * th
    rz = kz * th
    print('K Vector: ', kx, ky, kz)

    pose = np.array([T[0, 3], T[1, 3], T[2, 3], rx, ry, rz])
    return pose


def rotvec_2_T(xyzrxryrz):
    """
    pose with rotation vector representation to homogeneous trafo matrix
    """
    T = np.zeros((4, 4))
    # todo
    th = np.sqrt(xyzrxryrz[3]**2 + xyzrxryrz[4]**2 + xyzrxryrz[5]**2)
    kx = xyzrxryrz[3] / th
    ky = xyzrxryrz[4] / th
    kz = xyzrxryrz[5] / th

    v = 1 - np.cos(th)
    s = np.sin(th)
    c = np.cos(th)
     
    r11 = kx*kx*v + c
    r12 = kx*ky*v - kz*s
    r13 = kx*kz*v + ky*s

    r21 = ky*kx*v + kz*s
    r22 = ky*ky*v + c
    r23 = ky*kz*v - kx*s

    r31 = kz*kx*v - ky*s
    r32 = kz*ky*v + kx*s
    r33 = kz*kz*v + c

    T = np.array([[r11, r12, r13, xyzrxryrz[0]],
                  [r21, r22, r23, xyzrxryrz[1]],
                  [r31, r32, r33, xyzrxryrz[2]],
                  [0  ,   0,   0,  1]])

    return T


def T_2_rpy(T):
    """
    homogeneous trafo matrix to pose with roll-pitch-yaw x,y,z,r,p,y
    """
    pose = np.zeros((6))
    # todo
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    
    b = np.arctan2(-T[2, 0], np.sqrt(T[0, 0]**2 + T[1, 0]**2))
    a = np.arctan2(T[1, 0]/np.cos(b), T[0, 0]/np.cos(b))
    g = np.arctan2(T[2, 1]/np.cos(b), T[2, 2]/np.cos(b))
    pose = np.array([x, y, z, a, b, g])
    return pose


def rpy_2_T(xyzrpy):
    """
    pose with roll-pitch-yaw to homogeneous trafo matrix
    """
    T = np.zeros((4, 4))
    # todo
    a = xyzrpy[3]
    b = xyzrpy[4]
    g = xyzrpy[5]

    r11 = np.cos(a)*np.cos(b)
    r12 = np.cos(a)*np.sin(b)*np.sin(g) - np.sin(a)*np.cos(g)
    r13 = np.cos(a)*np.sin(b)*np.cos(g) + np.sin(a)*np.sin(g)

    r21 = np.sin(a)*np.cos(b)
    r22 = np.sin(a)*np.sin(b)*np.sin(g) + np.cos(a)*np.cos(g)
    r23 = np.sin(a)*np.sin(b)*np.cos(g) - np.cos(a)*np.sin(g)

    r31 = -np.sin(b)
    r32 = np.cos(b)*np.sin(g)
    r33 = np.cos(b)*np.cos(g)

    T = np.array([[r11, r12, r13, xyzrpy[0]], 
                  [r21, r22, r23, xyzrpy[1]],
                  [r31, r32, r33, xyzrpy[2]], 
                  [0  ,   0,   0, 1]])

    return T

def dh(alpha, a, d, theta):
    """
    
    Denavit-Hartenberg (classic) 
    """
    T = np.zeros(4)
    # todo
    r00 = 0 if theta == np.pi/2 or theta == -np.pi/2 else np.cos(theta)
    r01 = 0 if alpha == np.pi/2 or alpha == -np.pi/2 else -np.sin(theta)*np.cos(alpha)
    r02 = np.sin(theta)*np.sin(alpha)
    r03 = 0 if theta == np.pi/2 or theta == -np.pi/2 else a * np.cos(theta)

    r10 = np.sin(theta)
    r11 = 0 if theta == np.pi/2 or alpha == np.pi/2 or theta == -np.pi/2 or alpha == -np.pi/2 else np.cos(theta)*np.cos(alpha)
    r12 = 0 if theta == np.pi/2 or theta == -np.pi/2 else -np.cos(theta)*np.sin(alpha)
    r13 = a * np.sin(theta)

    r21 = np.sin(alpha)
    r22 = 0 if alpha == np.pi/2 or alpha == -np.pi/2 else np.cos(alpha)

    T = np.array([[r00, r01, r02, r03],
                  [r10, r11, r12, r13],
                  [  0, r21, r22,   d],
                  [  0,   0,   0,   1]])

    return T

def dhm(alpha, a, d, theta):
    """
    Denavit-Hartenberg (modified)
    """
    T = np.zeros(4)
    # todo

    r00 = np.cos(theta)
    r01 = -np.sin(theta) 

    r10 = np.sin(theta)*np.cos(alpha)
    r11 = np.cos(theta)*np.cos(alpha)
    r12 = -np.sin(alpha)
    r13 = -d * np.sin(alpha)

    r20 = np.sin(theta)*np.sin(alpha)
    r21 = np.cos(theta)*np.sin(alpha)
    r22 = np.cos(alpha)
    r23 = d * np.cos(alpha)

    T = np.array([[r00, r01,   0,   a],
                  [r10, r11, r12, r13],
                  [r20, r21, r22, r23],
                  [  0,   0,   0,   1]])

    return T

def fk_ur_dh(dh_para, q):
    """
    Forward Kinematics for UR type robots
    alpha = 0, a = 1, d = 2, theta = 3
    """
    T_0_6 = np.eye(4)
    # todo
    for n, qn in zip(dh_para, q):
      T_0_6 = T_0_6 @ dh(n[0], n[1], n[2], qn)
    return T_0_6
    

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def rotx(a):
    """
    Rotation about x axis
    """
    ca = np.cos(a)
    sa = np.sin(a)
    T = np.array([(1, 0, 0, 0), (0, ca, -sa, 0), (0, sa, ca, 0), (0, 0, 0, 1)])
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
    #Translation
    pose[0] = T[0][3] #x
    pose[1] = T[1][3] #y
    pose[2] = T[2][3] #z
    #Rotation
    theta = np.arccos((T[0,0] + T[1,1] +T[2,2] -1) / 2)
    #print("Theta : T to Rotvec: ", theta)
    #print("#######################\n",  (T[2,1] - T[1,2]), "#################\n")
    K = 1/(2*np.sin(theta)) * np.array([[T[2,1] - T[1,2]], [T[0,2] - T[2,0]], [T[1,0] - T[0,1]]])*theta
    pose[3::] = K.T
    #return [x y z rx ry rz]
    return pose


def rotvec_2_T(xyzrxryrz):
    """
    pose with rotation vector representation to homogeneous trafo matrix
    """
    #                           Translation     Drehw.-x, Drehw.-y, Drehw.-z
    #Uebergabe als Vektor z.b. [1,2,3,          0.1,        0.2,       0.3]
    T = np.eye(4)
    #Nebenrechnungen
    
    theta = np.sqrt(xyzrxryrz[3]**2 + xyzrxryrz[4]**2 + xyzrxryrz[5]**2 )
    print("Theta: Rotvec to T: ", theta)
    kx = xyzrxryrz[3]/theta
    ky = xyzrxryrz[4]/theta
    kz = xyzrxryrz[5]/theta
    
    vtheta = 1 - np.cos(theta)
    ctheta = np.cos(theta)
    stheta = np.sin(theta)
    
    #RotMat bauen
    RotM = np.array([[kx*kx*vtheta+ctheta, kx*ky*vtheta-kz*stheta, kx*kz*vtheta + ky*stheta],
                    [kx*ky*vtheta+kz*stheta, ky*ky*vtheta+ctheta, ky*kz*vtheta-kx*stheta],
                    [kx*kz*vtheta-ky*stheta, ky*kz*vtheta+kx*stheta, kz*kz*vtheta+ctheta]])
    #Rotationsmatrix in Transformationsmatrix uebertragen
    T[0:3, 0:3] = RotM
    #Verschiebung xyz uebertragen
    T[0:3, 3] = xyzrxryrz[:3]
    
    return T


def T_2_rpy(T):
    """
    homogeneous trafo matrix to pose with roll-pitch-yaw x,y,z,r,p,y
    """
    pose = np.zeros((6))
    #Translation
    pose[0] = T[0][3] #x
    pose[1] = T[1][3] #y
    pose[2] = T[2][3] #z
    #beta = roll
    #alpha = pith
    #gamma = yaw
    beta = np.arctan2(-T[2,0], np.sqrt(T[0,0]**2 + T[1,0]**2))
    alpha = np.arctan2(T[1,0]/np.cos(beta), T[0,0]/np.cos(beta))
    gamma = np.arctan2(T[2,1]/np.cos(beta), T[2,2]/np.cos(beta))
    pose[3] = beta
    pose[4] = alpha
    pose[5] = gamma
    #return [x y z roll pith yaw]
    return pose


def rpy_2_T(xyzrpy):
    """
    pose with roll-pitch-yaw to homogeneous trafo matrix
    """
    #                           Translation     roll,     pitch,       yaw
    #Uebergabe als Vektor z.b. [1,2,3,          0.1,        0.2,       0.3]
    T = np.eye(4)
    #Nebenrechnungen
    #beta = roll
    #alpha = pith
    #gamma = yaw
    alpha = xyzrpy[4]
    beta = xyzrpy[3]
    gamma = xyzrpy[5]
    
    #RotMat bauen
    # Rotz(alpha) * (Roty(beta) * Rotx(gamma))
    RotM = np.dot(rotz(alpha), np.dot(roty(beta), rotx(gamma)))
    #Rotationsmatrix in Transformationsmatrix uebertragen
    T[0:4, 0:4] = RotM
    #Verschiebung xyz uebertragen
    T[0:3, 3] = xyzrpy[:3]
    
    return T

def plotCS(axis, T_0_A, size=1, linewidth=3):
    #                   Ursprung     X-Achse   Y-Achse    Z-Achse
    p_in_A = np.array([[0,0,0,1], [0.1,0,0,1], [0,0.1,0,1], [0,0,0.1,1]]).T
    
    p_in_0 = np.dot(T_0_A, p_in_A)

    # x-axis
    # Generiert die Vektoren der neuen Kooardinatenachsen [Erste Zeile, [Ursprung, Pfeilende des X-"Pfeil"]]
    axis.plot(p_in_0[0, [0,1]], p_in_0[1, [0,1]], p_in_0[2, [0,1]], 'r-', linewidth=linewidth)
    # Y-axis
    # Generiert die Vektoren der neuen Kooardinatenachsen [Erste Zeile, [Ursprung, Pfeilende des Y-"Pfeil"]]
    axis.plot(p_in_0[0, [0,2]], p_in_0[1, [0,2]], p_in_0[2, [0,2]], 'g-', linewidth=linewidth)
    # Z-axis
    # Generiert die Vektoren der neuen Kooardinatenachsen [Erste Zeile, [Ursprung, Pfeilende des Z-"Pfeil"]]
    axis.plot(p_in_0[0, [0,3]], p_in_0[1, [0,3]], p_in_0[2, [0,3]], 'b-', linewidth=linewidth)


def dh(alpha, a, d, theta):
    """
    Denavit-Hartenberg (classic)
    """
    # Umrechnung in Milimeter
    #a = a*1000
    #d = d*1000
    T = np.eye(4)
    T = np.dot(rotz(theta), np.dot(transl(0, 0, d).T, np.dot(transl(a, 0, 0).T, rotx(alpha))))
    '''
    -------------------Alternativ mit ausmultiplizierter Matrix-------------------
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    ct = np.cos(theta)
    st = np.sin(theta)
    T = np.array([[ct,  -st*ca, st*sa,  a*ct],
                  [st,  ct*ca,  -ct*sa, a*st],
                  [0,   sa,     ca,     d],
                  [0,   0,      0,      1]])
    '''
    return T


def dhm(alpha, a, d, theta):
    """
    Denavit-Hartenberg (modified)
    """
    T = np.eye(4)
    # todo
    return T


def fk_ur(dh_para, q):
    """
    Forward Kinematics for UR type robots
    """

   
    
    T_0_6 = np.eye(4)
    
    #Laufe von Arraygroeße bis 0
    for i in range(dh_para.shape[0]-1, -1, -1):
        #print(i)
        DHneu = dh(dh_para[i][0], dh_para[i][1], dh_para[i][2], q[i])            
        T_0_6 = np.dot(DHneu, T_0_6)
    '''
    for i in dh_para:
        DHneu = dh(i[0], i[1], i[2], q[temp])
        T_0_6 = np.dot(DHneu, T_0_6)
    '''
    return T_0_6


def bk_ur(dh_para,TCP):
    """
    Backward Kinematics for UR type robots
    
    """
    if 1:
        result = np.zeros((8,6))
        #---------------------------
        #            q1
        #---------------------------
        T_0_6 = rotvec_2_T(TCP)
        VecO5 = np.array([0,0,-dh_para[5,2],1])
        O5 = np.dot(T_0_6,VecO5)
        #print("O5: ", O5)
        
        #Hilfsvariablen berechnen 
        alpha1 = np.arctan2(O5[1],O5[0])
        R = np.sqrt(O5[0]**2 + O5[1]**2)
        alpha2 = np.arccos(dh_para[3,2]/R)
        q1 = []
        
        q1.append(alpha1 + alpha2 + np.pi/2)
        result[0:4,0]= q1[0]/np.pi * 180
        q1.append(alpha1 - alpha2 + np.pi/2)
        result[4:8,0]=q1[1]/np.pi * 180
        #print("Q1: ", q1)
    
        #---------------------------
        #            q5
        #---------------------------
        q5 = []
        for i in range(2):
            s1 = np.sin(q1[i])
            c1 = np.cos(q1[i])
            q5.append(np.arccos((TCP[0]*s1 - TCP[1]*c1 - dh_para[3,2]) / dh_para[5,2]))
            result[4*i:i*4+2,4]= q5[i*2]/np.pi * 180
            q5.append(-np.arccos((TCP[0]*s1 - TCP[1]*c1 - dh_para[3,2]) / dh_para[5,2]))
            result[4*i+2:i*4+4,4]= q5[i*2+1]/np.pi * 180
        #print("Q5: ", q5)
        
        #---------------------------
        #            q6
        #---------------------------
        q6 = []
        for i in range(0,4):
            if i<2: #Fallunterscheidung
                s1 = np.sin(q1[0])
                c1 = np.cos(q1[0])
            else: 
                s1 = np.sin(q1[1])
                c1 = np.cos(q1[1])
            s5 = np.sin(q5[i])  
            q6.append(np.arctan2( (-T_0_6[0,1]*s1 + T_0_6[1,1]*c1)/s5, (T_0_6[0,0]*s1 - T_0_6[1,0]*c1)/s5 ))
            result[2*i:i*2+2,5]= q6[i]/np.pi * 180
        #print("Q6: ", q6)
        
        #---------------------------
        #          T_1_4
        #---------------------------
        for i in range(0,4):
            if i<2: #Fallunterscheidung
                temp_q1 = q1[0]
            else:
                temp_q1 = q1[1]
  
            T_5_4 = Tinv(dh(dh_para[4,0], dh_para[4,1], dh_para[4,2], q5[i]))
            T_6_5 = Tinv(dh(dh_para[5,0], dh_para[5,1], dh_para[5,2], q6[i]))
            T_1_0 = Tinv(dh(dh_para[0,0], dh_para[0,1], dh_para[0,2], temp_q1))
            
            #T_5_4 = dh(dh_para[4,1], dh_para[4,2], dh_para[4,0], q5[i])
            #T_6_5 = dh(dh_para[5,1], dh_para[5,2], dh_para[5,0], q6[i])
            #T_1_0 = dh(dh_para[0,1], dh_para[0,2], dh_para[0,0], temp_q1)
            
            T_1_4 = np.dot(T_1_0, np.dot(T_0_6, np.dot(T_6_5, T_5_4)))
            #print("\n \n T_1_4 -", i, ": \n", T_1_4)
            

        
            O4 = T_2_rotvec(T_1_4)
            #print("\n\nO4: ", O4)
            x = O4[0]
            z = O4[1]
            
            rx = O4[3]
            ry = O4[4]
            rz = O4[5]
            #print("###### rx %f    ry %f     rz %f ######" %(rx, ry, rz))
            phi = np.arctan2(T_1_4[1,0], T_1_4[0,0])

             
            #print("x: %f y: %f phi: %f" %(x, z, phi))
            
            l1 = dh_para[1,1]
            l2 = dh_para[2,1]
            
            #---------------------------
            #            q3
            #---------------------------
            q3 = np.pi - np.arccos(-(x**2 + z**2 - l1**2 - l2**2) / (2* l1 * l2)) 
            #print("x^^2 + y^^2 <= l1 + l2: %f <= %f" %(x**2 + z**2, l1 + l2))
            
            q3_2 = -q3
            #print("Q3: ",q3/np.pi * 180, "Q3_2: ", q3_2/np.pi * 180)
            result[2*i,2]= q3/np.pi * 180
            result[2*i+1,2]= q3_2/np.pi * 180
            
            
            #---------------------------
            #            q2
            #---------------------------
            beta = np.arctan2(z,x)
            psi = np.arccos((x**2 + z**2 + l1**2 - l2**2) / (2*l1*np.sqrt(x**2 + z**2)))
            while psi > np.pi: psi -= np.pi
            #print("beta: %f psi: %f" %(beta/np.pi * 180, psi/np.pi * 180))
            q2 = psi + beta
            while q2 > np.pi: q2 -= 2*np.pi
            q2_2 = beta - psi
            while q2_2 > np.pi: q2_2 -= 2*np.pi
            #print("Q2: ", q2/np.pi * 180, "Q2_2: ", q2_2/np.pi * 180)  
            result[2*i,1]= q2/np.pi * 180
            result[2*i+1,1]= q2_2/np.pi * 180
             
            
            #---------------------------
            #            q4
            #---------------------------
            q4 = phi - q3 - q2
            while q4 > np.pi: q4 -= 2*np.pi
            q4_2 = phi - q3_2 - q2_2
            while q4_2 > np.pi: q4_2 -= 2*np.pi
            #print("Q4: ", q4/np.pi * 180, "Q4_2: ", q4_2/np.pi * 180)
            result[2*i,3]= q4/np.pi * 180
            result[2*i+1,3]= q4_2/np.pi * 180
   
        
   
        
    return result

def jacobi(dh_para, q):
    J = np.eye(6)
    T_0_i = np.eye(4)
    
    #Vorwaertskinematik
    T_0_6 = fk_ur(dh_para, q)
    p = T_0_6[0:3, 3]
    
    for i in range(6):
        T = dh(dh_para[i][0],dh_para[i][1],dh_para[i][2], q[i])
        T_0_i = np.dot(T_0_i, T)
        z_i = T_0_i[0:3, 2]
        p_i = T_0_i[0:3, 3]
        r = p - p_i
        J[0:3, i] = np.cross(z_i, r) #J_trans
        J[3:6, i] = z_i#J_Rot
        
    
    
    
    return J











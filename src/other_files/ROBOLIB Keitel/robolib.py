# -*- coding: utf-8 -*-
"""
Bitte keine Kommentare, Variablen- u. Funktionsnamen oder Funktionsbeschreibungstexte übernehmen!


"""

import numpy as np
from copy import deepcopy

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
    #T = np.array([(1, 0, 0, x), (0, 1, 0, y), (0, 0, 1, z), (0, 0, 0, 1)])
    return T


def Tinv(T):
    """
    Inverse of homogeneous trafo matrix
    """
    # Anpassung für Numpy:
    T = np.array(T)
    
    R = T[0:3, 0:3]
    Ri = R.transpose()
    Ti = np.eye(4)
    Ti[0:3, 0:3] = Ri
    Ti[0:3, 3:4] = -(Ri @ T[0:3, 3:4])
    return Ti


def T_2_rotvec(T):
    """
    Transformationsmatrix zu Pose
    
    Rotationsachse und Winkel 
    
    homogeneous trafo matrix to rotation vector representation
    T (4x4) --> pose xyzrxryrz (1x6)
    """
    pose = np.zeros((6))

    theta = np.arccos((T[0,0] + T[1,1] + T[2,2] - 1)/2)
    
    faktor = 1 / (2*np.sin(theta))
    
    kx = faktor * (T[2,1] - T[1,2])
    ky = faktor * (T[0,2] - T[2,0])
    kz = faktor * (T[1,0] - T[0,1])

    if (kx*theta < 0):
        theta = theta - 2*np.pi
    
    pose[0] = T[0,3]        # x
    pose[1] = T[1,3]        # y
    pose[2] = T[2,3]        # z
    pose[3] = kx * theta    # rx
    pose[4] = ky * theta    # ry
    pose[5] = kz * theta    # rz
    
    return pose


def rotvec_2_T(xyzrxryrz):
    """
    Pose zu Transformationsmatrix
    
    Rotationsachse und Winkel
    
    pose with rotation vector representation to homogeneous trafo matrix
    pose (x,y,z,rx,ry,rz) (1x6) --> T (4x4)
    """
    T = np.zeros((4, 4))

    x = xyzrxryrz[0]
    y = xyzrxryrz[1]
    z = xyzrxryrz[2]
    rx = xyzrxryrz[3]
    ry = xyzrxryrz[4]
    rz = xyzrxryrz[5]

    theta = np.sqrt(rx**2 + ry**2 + rz**2)
    
    kx = rx / theta
    ky = ry / theta
    kz = rz / theta 
    
    vt = 1 - np.cos(theta)
    ct = np.cos(theta)
    st = np.sin(theta)
    
    T[0] = [kx*kx*vt+ct,      kx*ky*vt-kz*st,   kx*kz*vt+ky*st,     x]
    T[1] = [kx*ky*vt+kz*st,   ky*ky*vt+ct,      ky*kz*vt-kx*st,     y]
    T[2] = [kx*kz*vt-ky*st,   ky*kz*vt+kx*st,   kz*kz*vt+ct,        z]
    T[3] = [0,                0,                0,                  1]
    
    return T


def T_2_rpy(T):
    """
    Winkeldarstellung
    
    homogeneous trafo matrix to pose with roll-pitch-yaw x,y,z,r,p,y
    T (4x4) --> pose (x,y,z,r,p,y)
    """
    pose = np.zeros((6))
    
    pose[0] = T[0,3]    # x
    pose[1] = T[1,3]    # y
    pose[2] = T[2,3]    # z
    
    beta = np.arctan2(-T[2,0], np.sqrt(T[0,0]**2 + T[1,0]**2))   # atan2(-r31, sqrt(r11^2 + r21^2))
    cosbeta = np.cos(beta)
    alpha = np.arctan2(T[1,0]/cosbeta, T[0,0]/cosbeta)
    gamma = np.arctan2(T[2,1]/cosbeta, T[2,2]/cosbeta)
    
    pose[3] = alpha # roll
    pose[4] = beta  # pitch
    pose[5] = gamma # yaw
    
    return pose


def rpy_2_T(xyzrpy):
    """
    Winkeldarstellung
    
    pose with roll-pitch-yaw to homogeneous trafo matrix
    pose (x,y,z, r,p,y) --> T (4x4)
    """
    T = np.zeros((4, 4))

    x = xyzrpy[0]
    y = xyzrpy[1]
    z = xyzrpy[2]
    alpha = xyzrpy[3]
    beta  = xyzrpy[4]
    gamma = xyzrpy[5]
    
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    cb = np.cos(beta)
    sb = np.sin(beta)
    cg = np.cos(gamma)
    sg = np.sin(gamma)
    
    # Rotationsmatrix mit homogener Erweiterung
    R_A_B_t = np.array([(ca*cb,   (ca*sb*sg)-(sa*cg),     (ca*sb*cg)+(sa*sg), x),
                      (sa*cb,   (sa*sb*sg)+(ca*cg),     (sa*sb*cg)-(ca*sg), y),
                      (-sb,     cb*sg,                  cb*cg,              z),
                      (0,       0,      0,      1)])
    
    T = R_A_B_t
    
    return T


def dh(alpha, a, d, theta):
    """
    Denavit-Hartenberg (classic)
    Berechnung einer homogenen Transformationsmatrix bei gegebenen DH-Parametern (klassisch)
    alpha: Verwindung. Rotation um die xn-Achse damit zn-1 auf zn-Achse liegt
        a: Armlänge. Translation entlang xn-Achse bis zum Ursprung n System
        d: Gelenkabstand. Translation entlang zn-1 Achse bis Schnittpunkt von zn-1 und xn Achse
    theta: Gelenkwinkel. Rotation um die zn-1 Achse damit xn-1 auf xn-Achse liegt
    
    Abfolge: 
        1. theta_n
        2. d_n
        3. a_n
        4. alpha_n
        
        T_n-1_n = Rot(z_n-1, theta_n)*Trans(z_n-1, d_n)*Trans(x_n, a_n)*Rot(x_n, alpha_n)
    """
    T = np.zeros((4,4))
    
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = [(ct, -st*ca, st*sa,  a*ct),
         (st, ct*ca,  -ct*sa, a*st),
         (0,  sa,     ca,     d),
         (0,  0,      0,      1)]
    
    return T


def dhm(alpha, a, d, theta):
    """
    Denavit-Hartenberg (modified)
    Berechnung einer homogenen Transformationsmatrix bei gegebenen DH-Parametern (modifiziert)
    
    Abfolge:
        1. Armlänge a_n-1
        2. Verwindung alpha_n-1
        3. Gelenkabstand d_n
        4. Gelenkwinkel theta_n
    """
    T = np.zeros((4,4))
    
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)
    
    T = [(ct,   -st,        0,      a),
         (st*ca, ct*ca,     -sa,   -d*sa),
         (st*sa, ct*sa,     ca,     d*ca),
         (0,     0,         0,      1)]
    
    return T


def fk_ur(dh_para, q):
    """
    Forward Kinematics for UR type robots
    Berechnung der Vorwärtskinematik bei gegebener Tabelle von DH-Parametern (nutzt dh)
     dh_para = np.array([   alpha,     a,      d,      q_home_offset   ]).T
     
     Gelenkwinkel q[] in rad:
     q = np.array([10, -30, -100, -120, 30, 60]) / 180 * np.pi
     
     Rückgabe:
         Transformationsmatrix T_0_6
    """
    
    # Anzahl Gelenke, d.h. Anzahl an Transformationen
    n = len(q)
    
    # tiefe Kopie, damit Arrayinhalte nicht mehr die gleichen Objekte referenzieren und versehentlich geändert werden
    p = deepcopy(dh_para)  

    
    # Einsetzen des übergebenen Gelenkvektors q anstelle von theta
    p[:n, 3] = q # in der 3. Spalte ist nun theta bzw. q_home_offset
     
    # erstmaliger Aufruf der ersten DH-Transformationsmatrix 
    #          (alpha, a,      d,      theta)
    T_0_6 = dh(p[0,0], p[0,1], p[0,2], p[0,3]) 
    
    # weitere (5) Aufrufe:
    for i in range(1, n):
        T_0_6 = np.dot(T_0_6, dh(p[i,0], p[i,1], p[i,2], p[i,3]))
    
    return T_0_6


def remove_tcp(pose, tcp_offset=0):
    """
    Entfernt aus der übergebenen Pose (x,y,z,rx,ry,rz) den Offset aus dem 
    konfigurierten TCP-Werkzeug und gibt die bereinigte Pose zurück.
    Als TCP-Offset des Werkzeugs (Greifer, Stift, o.ä.) wird 
    die übergebene Liste tcp_offset verwendet
    
    """
    # zuerst: TCP-Konfiguration aus der pose herausrechnen
    # übergebene pose beinhaltet ja das eingestellte, stiftähnliche Werkzeug (den konfigurierten Tool Center Point)
    # eingestellter Tool Center Point: (X, Y, Z, rx, ry, rz):
    # np.array([0.02477, 0.003, 0.19922, 0, 0, 0])

    # Verwendung Default-Offset für Werkzeug, wie wir ihn als Gruppe eingestellt haben
    if type(tcp_offset)==int:
        if tcp_offset == 0:
            tcp_offset = np.array([0.02477, 0.003, 0.19922, 0., 0., 0.])

    #print("Pose mit TCP: ", pose)
    
    # Transformationsmatrix von Basis 0 bis Werkzeugspitze (Greifer, TCP):
    T_0_TCP = rotvec_2_T(pose)  
    # allg.:        T_0_TCP = T_0_6 * transl(tcp_offset)
    # daraus folgt: T_0_6 = inv(transl(tcp_offset) * inv(T_0_TCP))
    T_0_6 = np.dot(transl(tcp_offset[0], tcp_offset[1], tcp_offset[2]).T, np.linalg.inv(T_0_TCP))
    T_0_6 = Tinv(T_0_6)
    # Ableiten der Pose (ohne Werkzeug) aus der ermittelten T_0_6-Trafomatrix:
    pose = T_2_rotvec(T_0_6)
    
    #print("Neue Pose ohne TCP: ", pose)
    return pose

def add_tcp(pose, tcp_offset=0):
    """
    Fügt zu der übergebenen Pose (x,y,z,rx,ry,rz) den Offset aus dem 
    konfigurierten TCP-Werkzeug hinzu und gibt die Pose mit dem TCP-Werkzeug zurück.
    Als TCP-Offset des Werkzeugs (Greifer, Stift, o.ä.) wird 
    die übergebene Liste tcp_offset verwendet
    
    """
    # Verwendung Default-Offset für Werkzeug, wie wir ihn als Gruppe eingestellt haben
    if type(tcp_offset)==int:
        if tcp_offset == 0:
            tcp_offset = np.array([0.02477, 0.003, 0.19922, 0., 0., 0.])
    
    # Transformationsmatrix von Basis 0 bis 6:
    T_0_6 = rotvec_2_T(pose)  
    # allg.:        T_0_TCP = T_0_6 * transl(tcp_offset)
    T_0_TCP = np.dot(T_0_6 , transl(tcp_offset[0], tcp_offset[1], tcp_offset[2]).T)

    # Ableiten der Pose (mit Werkzeug) aus der ermittelten T_0_TCP-Trafomatrix:
    pose = T_2_rotvec(T_0_TCP)
    
    #print("Neue Pose mit TCP: ", pose)
    return pose

def rk_ur(dh_para, pose, conf=0, debug=0):
    """
    Rückwärtskinematik für UR3-Roboter
    Pose [x,y,z,rx,ry,rz] --> Gelenkwinkel q [°,°,°,°,°,°]  (bzw. in rad)
    
    Parameter:
        dh_para = np.array([   alpha,     a,      d,      q_home_offset   ]).T
        pose : Pose OHNE TCP-Tool-Offset [x, y, z, rx, ry, rz]
        conf : wählt die Lösung aus (0...7)
    
    Hinweis: 
        vor Verwendung mittels remove_tcp() den Offset aus der Pose entfernen, 
        damit sinnvolle Gelenkwinkel herauskommen
    """
    
    # Übergabeargument pose: (in m bzw. rad)
    x = pose[0]
    y = pose[1]
    z = pose[2]
    
    # Armlängen aus DH-Parameterliste
    l1 = dh_para[0,2]
    l2 = dh_para[1,1]
    l3 = dh_para[2,1]
    l4 = dh_para[3,2]
    l5 = dh_para[4,2]
    l6 = dh_para[5,2]
        
    # Abfolge aus Foliensatz Kapitel 05 Rückwärtskinematik, S. 23ff.
    # Berechnung von T_0_6 (dim: 4x4)
    T_0_6 = rotvec_2_T(pose)
    
    if debug:
        print("------T_0_6:")
        print(T_0_6)
    
    # Koordinaten von O5 im System 6
    o_6_5 = np.array([0, 0, -l6, 1])   # o_6_5 = [0, 0, -l6, 1] mit l6 = d6 aus der DH-Tabelle
    
    # Koordinaten von O5 im System 0
    o_0_5 = np.dot(T_0_6, o_6_5)
    if debug:
        print("------o_0_5:")
        print(o_0_5)
    
    # =======================================================================
    # Winkel q1
    alpha1 = np.arctan2(o_0_5[1], o_0_5[0]) # atan2(o_x, o_y)
    R = np.sqrt(o_0_5[0]**2 + o_0_5[1]**2)  # sqrt(o_x^2 + o_y^2)
    alpha2 = np.arccos(l4 / R)              # acos(l4 / R)
    
    # Unterscheidung conf
    if conf <= 3:
        q1 = alpha1 + alpha2 + np.pi/2        # erste Lösung für Gelenkwinkel q1
    else:
        q1 = alpha1 - alpha2 + np.pi/2        # zweite Lösung für Gelenkwinkel q1
    
    if debug:
        print("------q1: %6.2f° %6.2f " %(q1*180/np.pi, q1))
    
    # =======================================================================
    # Winkel q5
    s1 = np.sin(q1)
    c1 = np.cos(q1)
    
    # Unterscheidung conf
    if (conf == 0) or (conf == 2) or (conf == 4) or (conf == 6):
        q5 = + np.arccos((x*s1 - y*c1 - l4) / l6)   # acos((x*s1 - y*c1 - l4) / l6)
    else:
        q5 = - np.arccos((x*s1 - y*c1 - l4) / l6)   # - acos((x*s1 - y*c1 - l4) / l6)

    if debug:
        print("------q5: %6.2f° %6.2f" %(q5*180/np.pi, q5))


    # =======================================================================
    # Winkel q6
    s5 = np.sin(q5)
    c5 = np.cos(q5)

    zaehler1 = -T_0_6[0,1]*s1 + T_0_6[1,1]*c1
    zaehler2 = T_0_6[0,0]*s1 - T_0_6[1,0]*c1
    
    if (s5 == 0) or (round(zaehler1,6) == 0) and (round(zaehler2,6) == 0):  # Rundung von float-Zahlen nötig
        # q6 frei wählbar / System unterbestimmt / Gelenke 2,3,4,6 parallel (laut Paper: Hawkins-UR-Kinematik)
        q6 = 0.0
        # q6 ist undefiniert, wenn: 
        #   - beide Zähler == 0 oder
        #   - s5 == 0 (--> c5 = +/-1, d.h. die Gelenke 2, 3, 4, 6 sind alle parallel, Lösung ist unterbestimmt.)
        # --> q6 frei wählbar?
    else:
        # Berechnung q6-Gelenkwinkel
        q6 = np.arctan2(zaehler1/s5, zaehler2/s5) # atan2((-r12*s1 + r22*c1)/s5 , (r11*s1 - r21*c1)/s5 )
        # Anpassung (laut PDF mit 8 versch. Lösungen war q6 immer das positive Pendant)
        while q6 < 0:
            q6 += 2*np.pi
    
    if debug:
        print("------q6: %6.2f° %6.2f" %(q6*180/np.pi, q6))
    
    
    # =======================================================================
    # Berechnung von q2, q3, q4 über Ebenes Problem: T_1_4 = T_1_0 * T_0_6 * T_6_5 * T_5_4
    
    T_0_1 = dh(dh_para[0,0], dh_para[0,1], l1, q1)    # dh("alpha1", a1, l1, q1)
    # Aufruf: dh(alpha, a, d, theta)
    # dh_para:  np.array([alpha, a, d, q_home_offset]).T
    # T_0_6 ist ja schon seit Beginn bekannt...
    T_5_6 = dh(dh_para[5,0], dh_para[5,1], l6, q6)
    T_4_5 = dh(dh_para[4,0], dh_para[4,1], l5, q5)
    
    # Bilden der Inversen mit vorgegebener Tinv()-Funktion
    T_1_0 = Tinv(T_0_1)
    T_6_5 = Tinv(T_5_6)
    T_5_4 = Tinv(T_4_5)
    
    #T_1_0 = np.linalg.inv(T_0_1)
    #T_6_5 = np.linalg.inv(T_5_6)
    #T_5_4 = np.linalg.inv(T_4_5)
    
    # Berechnung von T_1_4 = T_1_0 * T_0_6 * T_6_5 * T_5_4
    T_1_6 = np.dot(T_1_0, T_0_6)
    T_1_5 = np.dot(T_1_6, T_6_5)
    T_1_4 = np.dot(T_1_5, T_5_4)
    
    # kurz:
    #T_1_4 = np.dot(np.dot(np.dot(T_1_0, T_0_6),T_6_5), T_5_4)
    
    if debug:
        print("------T_1_4:")
        print(T_1_4)
    

    # die letzten Einträge hinten in der T_1_4-Matrix
    x = T_1_4[0,3]
    y = T_1_4[1,3]
    if debug:
        print("x,y : letzte Einträge hinten in T14-matrix")
        print("------x: ", x)
        print("------y: ", y)
    # Alternative Extraktion von x, y:
    #print(T_2_rotvec(T_1_4)[0])
    #print(T_2_rotvec(T_1_4)[1])
    #print(T_1_4)
    
    # Berechnung phi (Extraktion aus T_1_4-Matrix)
    phi = np.arctan2(T_1_4[1,0], T_1_4[0,0])
    if debug:
        print("phi: ", phi)
    
    
    # Berechnung von q3
    # =======================================================================
    
    #gamma = np.arccos(-(x**2 + y**2 - l1**2 - l2**2)/(2*l1*l2))
    gamma = np.arccos(-(x**2 + y**2 - l2**2 - l3**2)/(2 * l2 * l3))
    q3 = np.pi - gamma
    # zwei Lösungen: Ellenbogen oben/unten
    #if (conf >= 2) and (conf <= 5):
    #if (conf >= 4):
    if (conf == 2) or (conf == 3) or (conf == 6) or (conf == 7):
        q3 = -q3
    if debug:
        print("------q3: %6.2f° %6.2f" % (q3*180/np.pi, q3))
    
    
    # Berechnung von q2
    # =======================================================================
    
    beta = np.arctan2(y, x)
    if debug:
        print("beta: ", beta)
    
    psi = np.arccos((x**2 + y**2 + l2**2 - l3**2)/(2*l2*np.sqrt(x**2 + y**2)))
    
    # Angleichen von psi: (0° =< psi =< 180°)
    while (psi > np.pi):
        psi -= np.pi
    
    if debug:
        print("psi: ", psi)
        
    q2 = psi + beta
    
    # Unterscheidung conf
   # if (conf >= 2) and (conf <= 5):
    if (q3 < 0):
       q2 = beta - psi

    # Angleichen von q2:
    while (q2 > np.pi):
        q2 -= 2*np.pi
    
    if debug:
        print("------q2: %6.2f° %6.2f" %(q2*180/np.pi, q2))
    
    
    
    # Berechnung von q4
    # =======================================================================
    
    q4 = phi - q3 - q2
    
    # Angleichen von q4:
    while (q4 > np.pi):
        q4 -= 2*np.pi
    
    if debug:
        print("------q4: %6.2f° %6.2f" %(q4*180/np.pi, q4))
    
    # Zusammensetzen des Gelenkwinkelvektors (joints):
    q = np.array([q1, q2, q3, q4, q5, q6])
    
    if debug:
        print(q*180/np.pi)
    
    return q



# Trajektorienbestimmung
# ---------------------------------------------------

def trj_schaltpunkte(v_max, a_max, Dq):
    """
    Funktion bestimmt anhand der übergebenen Argumente die Schaltzeitpunkte (in s)
    und ob Dreiecks- oder Trapeztrajektorie in Frage kommt
    
    v_max : maximale Geschwindigkeit in rad/s       UR3: 1.05 rad/s   | 0.25 m/s    (moveL)
    a_max : maximale Beschleunigung  in rad/s^2     UR3: 1.4  rad/s^2 | 1.2 m/s^2   (moveL)
    Dq    : Gelenkwinkel, der gefahren werden soll (in rad bzw. Verfahrweg in Meter)
    
    Rückgabeliste: [ dreieck(0):trapez(1), t_gesamt, t_s(1), (t_s2) ]
    
    Formeln aus Foliensatz Kapitel 06 der Vorlesung "Robotik"
    Ansatz 5.1: schnellstmögliche Trajektorie, Berechnung Gesamtzeit
    """
    if v_max==0 or a_max==0 or Dq==0:
        # keine Bewegung
        t_points = np.array([1,0,0,0])
    else:
        # Grenzwinkel für Unterscheidung Dreieck/Trapez
        q_grenz = (v_max**2) / a_max
        if Dq <= q_grenz:
            # Dreieck
            # Schaltzeitpunkt
            t_s = np.sqrt(abs(Dq / a_max))
            # Gesamtzeit
            t_ges = 2*t_s
            # Einbetten in Rückgabeliste
            t_points = [0, t_ges, t_s, 0]
        else:
            # Trapez (Dq > q_grenz)
            # 1. Schaltzeitpunkt
            t_s1 = v_max / a_max
            t_s2 = abs(Dq / v_max)
            t_ges = t_s1 + t_s2
            # Einbetten in Rückgabeliste
            t_points = [1, t_ges, t_s1, t_s2]
                        
    return t_points

def trj_tqva(v_max, a_max, Dq):
    """
    Funktion ermittelt den Trajektorienverlauf für Diskretisierungsschritte von 8 ms
    und gibt ihn als großes Array zurück.
    Es wird eine Fallunterscheidung zw. Dreiecks- und Trapezprofil gemacht
    
    Gegeben: max. Geschwindigkeit, max. Beschleunigung, Verfahrwinkel
    
    v_max : max. Geschwindigkeit in rad/s
    a_max : max. Beschleunigung in rad/s^2
    Dq    : Verfahrwinkel bzw. Verfahrweg in rad bzw. m
    
    Rückgabeliste: 
        [[ Zeitschritt (ms), Gelenkwinkel (°), Geschwindigkeit, Beschleunigung ],
         [ Zeitschritt (ms), Gelenkwinkel (°), Geschwindigkeit, Beschleunigung ], 
        ...]
    """
    # Kapitel 06, S. 20ff.
    # Ansatz 5.1
    
    # Entscheidung, ob überhaupt Achse verfahren werden soll:
    if np.round(Dq,6) == 0:
        # leeres Rückgabearray, damit Folgefunktionen dies ggf. entsprechend auffüllen können
        trj = []
        trj.append(np.zeros(4))
    else:
        # Falls Betrag( Dq ) > 0, dann mache alles Folgende:
            
        # zunächst positive Betrachtung, zuletzt werden die Werte ggf. invertiert
        Dq_positiv = True
        if Dq < 0:
            Dq_positiv = False
            Dq = abs(Dq)
        
        # Berechnung der Schaltzeitpunkte [dreieck/trapez, t_ges, t_s(1), (t_s2)]
        schaltpunkte = trj_schaltpunkte(v_max, a_max, Dq)   # Dq in rad
        dreieck = True
        if schaltpunkte[0] == 1:
            dreieck = False
            
        # Vorgabe UR, alle 8 ms 
        diskretisierung = 0.008 
        # Anzahl der nötigen Gesamtschritte ergibt sich aus der Gesamtzeit t_ges / 8 ms
        
        steps = int(np.ceil(schaltpunkte[1] / diskretisierung))
        #print("-----------steps: ", steps)
        
        # Dreieck-Trajektorie
        if dreieck:
            # Erzeugen leeres Grundgerüst [ Zeitschritt (ms), Gelenkwinkel (°), Geschwindigkeit, Beschleunigung ]
            trj = np.zeros((steps, 4))    
           
            # Geschwindigkeit am Schaltzeitpunkt
            v_s = np.sqrt(Dq * a_max)
            
            # Winkel zum Schaltzeitpunkt
            if a_max == 0:
                q_s = 0
            else:
                # Kapitel 06, S. 21 
                #q_s = 0.5 * (v_max**2 / a_max)
                q_s = 0.5*Dq
                
            for i in range(steps):
                # 1. Spalte: Zeitschritt
                t = i * diskretisierung         
                # Abschnitt A bis t_s
                if t <= schaltpunkte[2]:
                    q = (0.5 * a_max * t**2) # * 180 / np.pi  # Umrechnung in ° Grad
                    v = a_max * t
                    a = a_max
                    # Einbau in Array
                    trj[i] = [t, q, v, a]    
  
                # Abschnitt B bis Ende
                if t > schaltpunkte[2]:
                    q = q_s + (v_s*(t - schaltpunkte[2]) - 0.5*a_max*(t - schaltpunkte[2])**2)
                    v = 2*np.sqrt(Dq*a_max) - a_max*t
                    a = -a_max
                    # Einbau in Array, Indexverschiebung wg. vorherigem zusätzlichem Eintrag Dreieckspitze
                    trj[i] = [t, q, v, a]
        
        # Trapez-Trajektorie
        if not dreieck:
            # Erzeugen leeres Grundgerüst [ Zeitschritt (ms), Gelenkwinkel (°), Geschwindigkeit, Beschleunigung ]
            #print("----Steps: ", steps)
            trj = np.zeros((steps, 4))    
            # Kapitel 06, S.24
            if a_max == 0:
                q_s1 = 0
                q_s2 = 0
            else:
                # Winkel zum 1. Schaltzeitpunkt
                q_s1 = 0.5 * (v_max**2 / a_max)
                q_s2 = Dq - q_s1
            
            for i in range(steps):
                # 1. Spalte: Zeitschritt
                t = i * diskretisierung         
                # Abschnitt A bis t_s1
                if t <= schaltpunkte[2]:
                    q = (0.5 * a_max * t**2) # * 180 / np.pi  # Umrechnung in ° Grad
                    v = a_max * t
                    a = a_max
                    # Einbau in Array
                    trj[i] = [t, q, v, a]    

                # Abschnitt B bis t_s2
                if t > schaltpunkte[2] and t < schaltpunkte[3]:
                    q = q_s1 + v_max*(t - schaltpunkte[2])
                    v = v_max
                    a = 0
                    # Einbau in Array, Indexverschiebung wg. vorherigem zusätzlichem t_s1-Eintrag
                    trj[i] = [t, q, v, a]  

                # Abschnitt C bis Ende
                if t >= schaltpunkte[3]:
                    if Dq == 0:
                        q = 0
                    else:
                        q = q_s2 + (v_max + (a_max/v_max)*Dq)*(t - schaltpunkte[3]) - 0.5*a_max*(t**2 - (schaltpunkte[3])**2)
                    v = v_max - a_max*(t - schaltpunkte[3])
                    a = -a_max
                    # Einbau in Array, Indexverschiebung wg. vorherigen zusätzlichen t_s1 und t_s2-Einträgen
                    trj[i] = [t, q, v, a]
            
        if not Dq_positiv:
            # Die Werte der Trajektorienspalten q,v,a müssen invertiert werden, 
            # wenn ursprünglicher Winkel Dq negativ war
            trj[:,1:4] = - trj[:,1:4]
        
    return trj
    

def trj_v(t_ges, Dq, a_max=1.4):
    """
    Funktion berechnet Geschwindigkeit v_B (bei maximaler Beschleunigung), damit die 
    vorgegebene Gesamtzeit t_ges erreicht wird.
    
    t_ges : Gesamtzeit in s
    Dq    : Verfahrwinkel in rad
    
    Rückgabe: v_B
    """
    # Kapitel 06, S. 27
    # Ansatz 5.2
    
    t_s1 = 0.
    
    t_s1a = t_ges/2 + 0.5*np.sqrt(t_ges**2 - 4*(Dq/a_max))
    t_s1b = t_ges/2 - 0.5*np.sqrt(t_ges**2 - 4*(Dq/a_max))
    
    try:
        if t_s1a > 0 and t_s1a < t_s1b:
            t_s1 = t_s1a
        else:
            t_s1 = t_s1b
    except:
        t_s1 = 0
    
    v_B = a_max * t_s1
    
    return v_B


def trj_v_a_25(t_ges, Dq):
    """
    Funktion berechnet Geschwindigkeit v_B und Beschleunigung a_A, damit die 
    vorgegebene Gesamtzeit t_ges erreicht wird.
    Vorgabe: Beschleunigungsphase ist 25% der Gesamtzeit
    
    t_ges : Gesamtzeit in s
    Dq    : Verfahrwinkel in rad
    
    Rückgabeliste: [v_B, a_A]
    """
    # Kapitel 06, S. 29
    # Ansatz 5.3
    
    v_B = (16*Dq) / (12*t_ges)
    a_A = (16*Dq) / (3*t_ges**2)
    
    return np.array([v_B, a_A])

    
def trj_v_a(schaltpunkte, Dq):
    """
    Funktion berechnet Geschwindigkeit v_B und Beschleunigung a_A, damit die 
    vorgegebenen Schaltzeitpunkte (t_s1) eingehalten werden
    
    schaltpunkte : [ dreieck(0)/trapez(1), t_gesamt, t_s(1), (t_s2) ]
    Dq    : Verfahrwinkel in rad bzw. in Meter (moveL)
    
    Rückgabeliste: [v_B, a_A]
    """
    # Kapitel 06, S. 30
    # Ansatz 5.4
    
    t_ges = schaltpunkte[1]
    t_s1 = schaltpunkte[2]
    
    v_B = Dq / (t_ges - t_s1)
    a_A = Dq / (t_ges*t_s1 - t_s1**2)
    
    return np.array([v_B, a_A])


def trj_sync(Dq, v_max=0, a_max=0):
    """
    Funktion ermittelt die führende Achse (Achse mit der größten Gesamtzeit).
    --> Ansatz 5.1
    Die Schaltzeitpunkte werden dadurch festgelegt.
    Anschließend werden die Trajektorien der Folgeachsen mit diesen Schaltzeitpunkten
    berechnet 
    --> Ansatz 5.4
    
    v_max : [v1, v2, v3, v4, v5, v6]
    a_max : [a1, a2, a3, a4, a5, a6]
    Dq    : [q1, q2, q3, q4, q5, q6] Verfahrwinkel in rad
    
    Rückgabe: 
        alle synchronen Trajektorien als riesengroßes Array
    """
    # Kapitel 06, S. 31
    # Ansatz 5.5 -> 5.1 -> 5.4: Schnellstmögliche Bewegung 
    
    # Behandlung der Vorbelegung für v_max- und a_max-Arrays
    # Standardwerte für moveJ !!! (v = 1.05 rad/s, a = 1.4 rad/s^2)
    if type(v_max)==int:
        if v_max == 0:
            v_max = np.array([1.05,1.05,1.05,1.05,1.05,1.05])
    if type(a_max)==int:
        if a_max == 0:
            a_max = np.array([1.4, 1.4, 1.4, 1.4, 1.4, 1.4])

    # Ermitteln des führenden (langsamsten) Gelenks (Leitachse)
    # Beginn mit der ersten (0.) Achse
    leitachse = 0
    t_ges_max = trj_schaltpunkte(v_max[0], a_max[0], Dq[0])[1]
    
    for achse in range(1,6):
        t_ges_achse = trj_schaltpunkte(v_max[achse], a_max[achse], Dq[achse])[1]
        if t_ges_achse > t_ges_max:
            # wenn Gesamtzeit der aktuellen Achse größer als die vorherige Gesamtzeit ist:
            t_ges_max = t_ges_achse
            # neue führende Achse:
            leitachse = achse 
    
    # (erneute) Berechnung ALLER Schaltzeitpunkte der langsamsten Achse
    schaltpunkte_max = trj_schaltpunkte(v_max[leitachse], a_max[leitachse], Dq[leitachse])
    
    # Die führende Achse ist:
    #print("Leitachse bzw. -koordinate (1...6): %d. Achse (moveJ) bzw. Koordinate (moveL)" % (leitachse+1))
    
    # Berechnung der jeweiligen Geschwindigkeiten und Beschleunigungen aller Achsen mit den 
    # Schaltpunkten der langsamsten Achse
    arr_v_a = np.zeros((6,2))
    for achse in range(6):
        arr_v_a[achse,:] = trj_v_a(schaltpunkte_max, Dq[achse])
    
    #print("----arr_v_a: ", arr_v_a[4])
    #print("----arr_v_a: ", Dq[4])
    
    # Aufbau von arr_v_a:
    # [[v0, a0],
    #  [v1, a1],
    #  [v2, a2], ...
        
    # Berechnung der Trajektorien für jede Achse (ggf. unterschiedliche Array-Shapes!)
    trj_0 = trj_tqva(arr_v_a[0,0], arr_v_a[0,1], Dq[0])
    trj_1 = trj_tqva(arr_v_a[1,0], arr_v_a[1,1], Dq[1])
    trj_2 = trj_tqva(arr_v_a[2,0], arr_v_a[2,1], Dq[2])
    trj_3 = trj_tqva(arr_v_a[3,0], arr_v_a[3,1], Dq[3])
    trj_4 = trj_tqva(arr_v_a[4,0], arr_v_a[4,1], Dq[4])
    trj_5 = trj_tqva(arr_v_a[5,0], arr_v_a[5,1], Dq[5])
    
    #print(trj_2)
    
    # ----------------
    # Werden einzelne Achsen nicht verfahren (Dq = 0), so sind deren Einzelachsen-
    # Trajektorienarrays leer.
    # solche "leeren" Arrays werden hier auf die selbe Länge gebracht wie die "gefüllten" Arrays
    merker_volle_arrays = [True, True, True, True, True, True]
    
    n_max_zeilen = 0    # höchste Anzahl Zeilen
    n_max_achse = 0     # Achse mit vollster Trajektorie
    
    # Zeilenanzahl der Einzeltrajektorien
    n_zeilen = np.array([len(trj_0), len(trj_1), len(trj_2), len(trj_3), len(trj_4), len(trj_5)])
    
    # Ermitteln der höchsten Zeilenlänge aller Trj-Arrays und der entsprechenden Achse
    # Ermitteln der "leeren" Arrays
    for i,laenge in enumerate(n_zeilen):
        if laenge > n_max_zeilen:
            # Setzen der neuen max. Zeilenlänge
            n_max_zeilen = laenge
            # Setzen der neuen vollsten Trajektorien
            n_max_achse = i
        # handelt es sich um "leeres" Array?
        merker_volle_arrays[i] = (laenge >= 5)
    
    #print("Zeileneinträge: ", n_max_zeilen)
    print("Achse bzw. Koordinate bewegt sich (True): ", merker_volle_arrays)
    
    # Festlegen der vollsten Trajektorie
    if n_max_achse == 0:
        trj_max = trj_0
    if n_max_achse == 1:
        trj_max = trj_1
    if n_max_achse == 2:
        trj_max = trj_2
    if n_max_achse == 3:
        trj_max = trj_3
    if n_max_achse == 4:
        trj_max = trj_4
    if n_max_achse == 5:
        trj_max = trj_5
    
    # trj_0
    if not merker_volle_arrays[0]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_0) < n_max_zeilen:
            trj_0 = np.append(trj_0, [trj_0[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_0[:,0] = deepcopy(trj_max[:,0])
        
    # trj_1
    if not merker_volle_arrays[1]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_1) < n_max_zeilen:
            trj_1 = np.append(trj_1, [trj_1[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_1[:,0] = deepcopy(trj_max[:,0])
        
    # trj_2
    if not merker_volle_arrays[2]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_2) < n_max_zeilen:
            trj_2 = np.append(trj_2, [trj_2[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_2[:,0] = deepcopy(trj_max[:,0])
        
    # trj_3
    if not merker_volle_arrays[3]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_3) < n_max_zeilen:
            trj_3 = np.append(trj_3, [trj_3[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_3[:,0] = deepcopy(trj_max[:,0])
        
    # trj_4
    if not merker_volle_arrays[4]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_4) < n_max_zeilen:
            trj_4 = np.append(trj_4, [trj_4[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_4[:,0] = deepcopy(trj_max[:,0])
        
    # trj_5
    if not merker_volle_arrays[5]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_5) < n_max_zeilen:
            trj_5 = np.append(trj_5, [trj_5[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_5[:,0] = deepcopy(trj_max[:,0])
    
    # Rückgabe aller synchronen Trajektorien als riesengroßes (gleichmäßiges) Array
    return np.array([trj_0, trj_1, trj_2, trj_3, trj_4, trj_5])


def trj_sync_t(Dq, t_ges, v_max=0, a_max=0):
    """
    synchrone Bewegung aller Achsen
    mit Angabe der Gesamtzeit t_ges
    
    t_ges : Gesamtzeit in s
    v_max : [v1, v2, v3, v4, v5, v6]
    a_max : [a1, a2, a3, a4, a5, a6]
    Dq    : Verfahrwinkel in rad
    
    Rückgabe:
        alle synchronen Trajektorien als riesengroßes Array
    """
    # Kapitel 06, S. 31 bzw. S. 29
    # Ansatz 5.5 -> 5.3 -> 5.1: Bewegung mit Angabe der Gesamtzeit
    
    # Behandlung der Vorbelegung für v_max- und a_max-Arrays
    if type(v_max)==int:
        if v_max == 0:
            v_max = np.array([1.05,1.05,1.05,1.05,1.05,1.05])
    if type(a_max)==int:
        if a_max == 0:
            a_max = np.array([1.4, 1.4, 1.4, 1.4, 1.4, 1.4])
            

    # Ermitteln der Geschwindigkeits- und Beschleunigungswerte v_B und a_A 
    # für die einzelnen Achsen, damit die vorgegebene Zeit t_ges eingehalten werden kann
    arr_v_a = np.zeros((6,2))
    for achse in range(6):
        # VORSICHT!! 
        
        # in Vorlesung wurde gesagt, dass für moveJ mit Zeitvorgabe Ansatz 5.5 und 25% verwendet werden soll:
        #arr_v_a[achse,:] = trj_v_a_25(t_ges, Dq[achse])
        # Ergebnis: für Aufgabe a2 erscheint ein Trapezverlauf, im Labor gab es allerdings einen Dreiecksverlauf (wie auch für a1) ) 
        
        # Test für Aufgabe 2 mit max. Beschleunigung, Ansatz 5.2
        #arr_v_a[achse,:] = [trj_v(t_ges,Dq[achse], a_max[achse]),a_max[achse]]
        # Ergebnis: extrem lange, langsame Bewegung mit v=0.11427 rad/s, um auf die Zeit 3.9s zu kommen (max. kurze 
        #   Beschleunigung mit a=1.4 rad/s2)
        
        # Ansatz 5.4: 
        # Vorgabe eines Dreiecksprofils mit einzigem Schaltzeitpunkt bei der Hälfte der 3.9s
        #  [ dreieck(0):trapez(1), t_gesamt, t_s(1), (t_s2) ]
        schaltpunkte = [0, t_ges, t_ges/2] # quasi abgelesen aus echter Roboter-Bewegung im Labor
        arr_v_a[achse,:] = trj_v_a(schaltpunkte, Dq[achse])
        # --> Ergebnis: genauso wie auch im Labor!
        
    # Merker: Aufbau von arr_v_a:
    # [[v0, a0],
    #  [v1, a1],
    #  [v2, a2], ...
    
    # Rückgabewerte im Fehlerfall (werden ansonsten überschrieben)
    trj_0 = np.array([0, 0, 0, 0, 0, 0])
    trj_1 = np.array([0, 0, 0, 0, 0, 0])
    trj_2 = np.array([0, 0, 0, 0, 0, 0])
    trj_3 = np.array([0, 0, 0, 0, 0, 0])
    trj_4 = np.array([0, 0, 0, 0, 0, 0])
    trj_5 = np.array([0, 0, 0, 0, 0, 0])
    
    # Überprüfen, ob die gewünschten Winkel Dq[] in der geg. Zeit verfahren werden können
    # d.h. die errechneten v_B oder a_A müssen KLEINER GLEICH wie übergebene v_max oder a_max sein
    moeglich = True
    for a in range(6):
        # v_B <= v_max , a_A <= a_max
        moeglich = moeglich and (arr_v_a[a,0] <= v_max[a]) and (arr_v_a[a,1] <= a_max[a])
    
    
    if moeglich:
        # Zeiten bzw. a_max/v_max i.O.:
        # mit eben bestimmten v_B und a_A (als v_max und a_max) 
        # Berechnung der Trajektorien für jede Achse (Ansatz 5.1)
        trj_0 = trj_tqva(arr_v_a[0,0], arr_v_a[0,1], Dq[0])
        trj_1 = trj_tqva(arr_v_a[1,0], arr_v_a[1,1], Dq[1])
        trj_2 = trj_tqva(arr_v_a[2,0], arr_v_a[2,1], Dq[2])
        trj_3 = trj_tqva(arr_v_a[3,0], arr_v_a[3,1], Dq[3])
        trj_4 = trj_tqva(arr_v_a[4,0], arr_v_a[4,1], Dq[4])
        trj_5 = trj_tqva(arr_v_a[5,0], arr_v_a[5,1], Dq[5])
        
    if not moeglich:
        print("Problem in Funktion trj_sync_t(): \n--------\nDie übergebene Gesamtzeit t_ges ist zu klein! Die dafür nötigen Werte für v und a würden die Limits überschreiten.")

    # ----------------
    # Werden einzelne Achsen nicht verfahren (Dq = 0), so sind deren Einzelachsen-
    # Trajektorienarrays leer. 
    # Solche "leeren" Arrays werden hier auf die selbe Länge gebracht wie die "gefüllten" Arrays
    merker_volle_arrays = [True, True, True, True, True, True]
    
    n_max_zeilen = 0    # höchste Anzahl Zeilen
    n_max_achse = 0     # Achse mit vollster Trajektorie
    
    # Zeilenanzahl der Einzeltrajektorien
    n_zeilen = np.array([len(trj_0), len(trj_1), len(trj_2), len(trj_3), len(trj_4), len(trj_5)])
    
    # Ermitteln der höchsten Zeilenlänge aller Trj-Arrays und der entsprechenden Achse
    # Ermitteln der "leeren" Arrays
    for i,laenge in enumerate(n_zeilen):
        if laenge > n_max_zeilen:
            # Setzen der neuen max. Zeilenlänge
            n_max_zeilen = laenge
            # Setzen der neuen vollsten Trajektorien
            n_max_achse = i
        # handelt es sich um "leeres" Array?
        merker_volle_arrays[i] = (laenge >= 5)
    
    print("Zeileneinträge: ", n_max_zeilen)
    print("Achse bzw. Koordinate bewegt sich (True): ", merker_volle_arrays)
    
    # Festlegen der vollsten Trajektorie
    if n_max_achse == 0:
        trj_max = trj_0
    if n_max_achse == 1:
        trj_max = trj_1
    if n_max_achse == 2:
        trj_max = trj_2
    if n_max_achse == 3:
        trj_max = trj_3
    if n_max_achse == 4:
        trj_max = trj_4
    if n_max_achse == 5:
        trj_max = trj_5
    
    # trj_0
    if not merker_volle_arrays[0]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_0) < n_max_zeilen:
            trj_0 = np.append(trj_0, [trj_0[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_0[:,0] = deepcopy(trj_max[:,0])
        
    # trj_1
    if not merker_volle_arrays[1]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_1) < n_max_zeilen:
            trj_1 = np.append(trj_1, [trj_1[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_1[:,0] = deepcopy(trj_max[:,0])
        
    # trj_2
    if not merker_volle_arrays[2]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_2) < n_max_zeilen:
            trj_2 = np.append(trj_2, [trj_2[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_2[:,0] = deepcopy(trj_max[:,0])
        
    # trj_3
    if not merker_volle_arrays[3]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_3) < n_max_zeilen:
            trj_3 = np.append(trj_3, [trj_3[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_3[:,0] = deepcopy(trj_max[:,0])
        
    # trj_4
    if not merker_volle_arrays[4]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_4) < n_max_zeilen:
            trj_4 = np.append(trj_4, [trj_4[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_4[:,0] = deepcopy(trj_max[:,0])
        
    # trj_5
    if not merker_volle_arrays[5]:
        # Auffüllen des "leeren" Arrays bis zur Gesamtzeilenanzahl wie das größte Achsentrj-Array
        # die erste Zeile des Achsentrajektorienarrays wird einfach z.B. 530x unten drangesetzt
        while len(trj_5) < n_max_zeilen:
            trj_5 = np.append(trj_5, [trj_5[0]], axis=0)
        # in die erste (noch leere) Spalte wird nun der Zeitstempel (0.008, 0.016, 0.024...) einfügt
        # dazu wird einfach eine "volle" Zeitstempel-Spalte der "vollsten" Achse verwendet.
        trj_5[:,0] = deepcopy(trj_max[:,0])
            
    # -------------------------    

    # Rückgabe aller Trajektorien als großes Array
    return np.array([trj_0, trj_1, trj_2, trj_3, trj_4, trj_5])


def moveJ(qstart, qend, a_max=0, v_max=0, t=0):
    """
    Nachbildung der moveJ-Funktion
    qstart : Start Gelenkwinkel     [q1, q2, q3, q4, q5, q6] in rad
    qend   : Ende Gelenkwinkel      [q1, q2, q3, q4, q5, q6] in rad
    a_max  : max. Beschleunigung    [a1, a2, a3, a4, a5, a6] in rad/s^2
    v_max  : max. Geschwindigkeit   [v1, v2, v3, v4, v5, v6] in rad/s
    t      : Gesamtzeit in s
    
    Rückgabe: 
        Trajektorie
    """
    
    # Behandlung der Vorbelegung für v_max- und a_max-Arrays
    if type(v_max)==int:
        if v_max == 0:
            v_max = np.array([1.05,1.05,1.05,1.05,1.05,1.05])
    if type(a_max)==int:
        if a_max == 0:
            a_max = np.array([1.4, 1.4, 1.4, 1.4, 1.4, 1.4])
    
    # Differenzwinkel Dq 
    Dq = qend - qstart
    #print("--> Differenzwinkel: ", Dq)
    # später wird auf alle Gelenkwinkeleinträge qstart draufaddiert
    
    if t==0:
        # Berechnung der Achsentrajektorien ohne Zeitvorgabe
        TRJ = trj_sync(Dq, v_max, a_max)
    else:
        # Berechung der Achsentrajektorien MIT Zeitvorgabe
        TRJ = trj_sync_t(Dq, t, v_max, a_max)

    # Korrektur der Gelenkwinkel     
    # der Startwinkel in rad wird auf die q-Spalten aller 6 Achsen draufaddiert
    for i, achse in enumerate(TRJ):  
        achse[:,1] += qstart[i]
            
        
    # Option: Erweiterung um die Berechnung der TCP-Koordinaten und/oder TCP-Geschwindigkeiten
    #   Für Aufgabenstellung jedoch nicht relevant, daher wird dies weggelassen
        
    
    # TRJ ist nun komplexes nested Array: 
    # für jede Achse [t, q, v, a]
    
    # Umstrukturierung zu einem eindimensionalem Array
    # damit intuitiver drauf zugegriffen werden kann...
    # ...
    
    return TRJ
        
def moveL(pstart, pend, a_max=0, v_max=0, t=0, conf=6, dh_para=0):
    """
    Nachbildung der moveL-Funktion
    pstart : Start Pose     [x, y, z, rx, ry, rz] in m
    pend   : Ende Pose      [x, y, z, rx, ry, rz] in m
    a_max  : max. Beschleunigung    [a1, a2, a3, a4, a5, a6] in m/s^2
    v_max  : max. Geschwindigkeit   [v1, v2, v3, v4, v5, v6] in m/s
    t      : Gesamtzeit in s
    conf   : Angabe der Standardkonfiguration für Rückwärtskinematik (empirischer Wert...)
    
    Rückgabe: 
        Trajektorie
    """
    # Behandlung der Vorbelegung für v_max- und a_max-Arrays
    # Standardwerte für MOVEL-Bewegung (v = 0,25 m/s, a = 1,2 m/s^2)
    if type(v_max)==int:
        if v_max == 0:
            v_max = np.array([0.25, 0.25, 0.25, 0.25, 0.25, 0.25])
            #v_max = np.array([1.05, 1.05, 1.05, 1.05, 1.05, 1.05])
    if type(a_max)==int:
        if a_max == 0:
            a_max = np.array([1.2, 1.2, 1.2, 1.2, 1.2, 1.2])
    
    # Behandlung der Vorbelegung für dh_para
    if type(dh_para)==int:
        if dh_para == 0:
            # values from UR-Sim
            # /home/ur/ursim-current/.urcontrol/urcontrol.conf
            a = [0.00000, -0.24365, -0.21325, 0.00000, 0.00000, 0.0000]
            d = [0.1519, 0.00000, 0.00000, 0.11235, 0.08535, 0.0819]
            alpha = [1.570796327, 0, 0, 1.570796327, -1.570796327, 0]
            q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
            # Standard DH-Para
            dh_para = np.array([alpha, a, d, q_home_offset]).T
    
    # Entfernen des TCP-Tool-Offsets aus den Posen
    pstart_ohne = remove_tcp(pstart)
    print("poseA-ohne: ", pstart_ohne)
    pend_ohne = remove_tcp(pend)
    print("poseB-ohne: ", pend_ohne)
    
    # Verfahrwege (dx, dy, dz, drx, dry, drz) ohne TCP-Werkzeug
    # linearer Bahnvektor: Endkoordinaten - Startkoordinaten
    bahn_vektor = pend_ohne - pstart_ohne
    print("Verfahrwege moveL:", bahn_vektor)
    
    # Die Länge dieses Vektors entspricht sqrt(x^2 + y^2 + z^2)
    bahn_len = np.sqrt(bahn_vektor[0]**2 + bahn_vektor[1]**2 + bahn_vektor[2]**2)
    print("lineare Verfahrlänge für moveL beträgt: %6.2f mm." % (bahn_len * 1000))

    
    if t==0:
        # Berechnung der Posen ohne Zeitvorgabe
        TRJ = trj_sync(bahn_vektor, v_max, a_max)
    else:
        # Berechung der Posen MIT Zeitvorgabe
        TRJ = trj_sync_t(bahn_vektor, t, v_max, a_max)

    # Korrektur der Koordinaten     
    # die Werte der Startpose_ohneTCP werden auf alle Koordinaten draufaddiert
    for i, achse in enumerate(TRJ):  
        achse[:,1] += pstart_ohne[i]
    
    # --> das Array TRJ besteht nun aus den Koordinatentrajektorien 
    # der x,y,z-Achsen und der rx,ry,rz-Orientierung der Roboterspitze (ohne TCP-Werkzeug)
    # ____TRJ_0__________   ____TRJ_1___________  ...
    # t | Koord x | v | a   t | Koord. y | v | a
    
    # Nun werden aus jeder Pose (alle 8ms) via Rückwärtskinematik 
    # die entsprechenden Gelenkwinkel berechnet.
    
    rk_q = []
    # Anm.: zip() erstellt die jeweilige Pose pro Zeitschritt als iterierbaren Sechser-Tupel:
    #(0.008, 0.3724, -0.37398, 0.27822, 1.79, 0.711, -0.197)
    #(0.016, 0.3724, -0.37398, 0.2782584, 1.79, 0.711, -0.197)
    # Aufbau:       ( t (ms),   x,           y,           z,           rx,          ry,          rz         )
    for row in zip(TRJ[0][:,0], TRJ[0][:,1], TRJ[1][:,1], TRJ[2][:,1], TRJ[3][:,1], TRJ[4][:,1], TRJ[5][:,1]):
        # Pose ohne TCP-Werkzeug
        p = row[1:] 
        # Berechnung der Gelenkwinkel zu jeder Pose über Rückwärtskinematik
        q = rk_ur(dh_para, p, conf)  # conf = 6 
        # Die Pose wird so erweitert, dass nun auch wieder das TCP-Werkzeug berücksichtigt ist (--> TCP-Koordinaten)
        p_mit = add_tcp(p)
        # neuer Zeileneintrag besteht aus: [ t (ms), q:q1,q2..., pose_ohne:x,y,z,rx,ry,rz, p_mit:x,y,z,rx,ry,rz TCP-Koord ]  # 19
        eintrag = [ row[0], q[0], q[1], q[2], q[3], q[4], q[5], p[0], p[1], p[2], p[3], p[4], p[5], p_mit[0], p_mit[1], p_mit[2], p_mit[3], p_mit[4], p_mit[5]]
        rk_q.append(eintrag)
    
    va_q = []
    x = 0
    # Berechnung v und a für Gelenkwinkel
    for i,eintrag in enumerate(rk_q):
        q = eintrag[1:7]    # Gelenkwinkel q1,q2,q3,q4,q5,q6
        if i == 0:
            # Startwerte für erste Zeile (Zeitpunkt t = 0ms)
            qd  = np.array([0., 0., 0., 0., 0., 0.])
            qdd = np.array([0., 0., 0., 0., 0., 0.])
        else:
            zeitdifferenz = eintrag[0] - rk_q[i-1][0]
            # Gelenkgeschwindigkeit qd (in rad/s) qd = v = dq / dt     (wird an vorletzte (-2.) Stelle angehängt)
            # Berechnung aus der Differenz (aktueller Winkel - vorheriger Winkel) / Zeitdifferenz
            qd  = (np.array(q) - np.array(rk_q[i-1][1:7])) / zeitdifferenz 
            
            # Gelenkbeschleunigung qdd (in rad/s^2) qdd = a = dv / dt  (wird an letzte (-1.) Stelle angehängt)
            # Berechnung aus der Differenz (aktuelle Geschw - vorherige Geschwindigkeit) / Zeitdifferenz
            qdd = (qd - rk_q[i-1][-2]) / (zeitdifferenz)
        #rk_q[i].append(qd)
        #rk_q[i].append(qdd)
        va_q.append([qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], qdd[0], qdd[1], qdd[2], qdd[3], qdd[4], qdd[5]])  # 12
        x += 1

    # Mit Jacobi-Matrix die TCP-Geschwindigkeiten berechnen
    v_TCP = []
    for eintrag in zip(rk_q, va_q):
        q = eintrag[0][1:7]
        qd = eintrag[1][0:6] 
        J = jacobi_matrix(q, dh_para)
        v = np.dot(J, qd)
        v_TCP.append(v)         # 6
        #v_TCP.append(np.linalg.norm(v[0:2]))

    # v_TCP[] Aufbau: z.B. 130 Zeilen mit [tcpv1, tcpv2, tcpv3, tcpv4, tcpv5, tcpv6]

    # Kombinieren von rk_q, va_q und v_TCP zu einem sauberen, eindimensionalen Listenarray:
    res = []
    for row,element in enumerate(rk_q):
        # t, q6, pose_ohne6, pose_mitTCP 6
        res.append(element)
    for row,element in enumerate(va_q):
        for v in element:
            # qd 6, qdd 6
            res[row].append(v)
    for row,element in enumerate(v_TCP):
        for v in element:
            # vTCP 6
            res[row].append(v)
        
    # ---> Umformen in sauberes np.Array
    #   im Array rk_q: z.B. 130 Zeilen mit je 19 Einträgen:
    #   [ t (ms),  q in rad:q1,q2...,   pose_ohne:x,y,z,rx,ry,rz,   p_mit:x,y,z,rx,ry,rz TCP-Koord , [v: qd (rad/s)], [a: qdd (rad/s^2) ]]
    #rk_q = np.array(rk_q)  
    
    # Variablenname beschreibt die Abfolge von Werten für die Zeit t, Winkel q etc. :)
    # insgesamt 37 Einträge pro Listen-Zeile
    t1q6p6p6v6a6tcpv6 = np.array(res)

    return t1q6p6p6v6a6tcpv6
    
    

def jacobi_matrix(q, dh_para):   
    """
    Berechnung der Jacobi-Matrix gemäß Kapitel 07 "Jacobimatrizen"
    
    """
    # Verwendung Default-Offset für Werkzeug, wie wir ihn als Gruppe eingestellt haben
    tcp_offset = np.array([0.02477, 0.003, 0.19922, 0., 0., 0.])
    
    # Erzeugen des Grundgerüsts für die Jacobi-Matrix
    J = np.zeros((6, 6))
    
    # Trafomatrix von 0 zu TCP-Werkzeugspitze mit Translationsmatrix
    T_0_TCP = np.dot(fk_ur(dh_para, q), transl(tcp_offset[0], tcp_offset[1], tcp_offset[2]).T )
    
    #T_0_TCP = fk_ur(dh_para, q)
    
    #T_0_6 = fk_ur(dh_para, q)               # transformation matrix of the system (forward kinematics)
    #point_end = T_0_6[0:3, 3]               # calculate the TCP origin coordinates
    spitze = T_0_TCP[0:3,3]

    # Erzeugen einer Einheitsmatrix
    T_0_i = np.eye(4) 
    
    for i in range(6):
        if i == 0:
            T_0_i = T_0_i
        else:
            T = dh(dh_para[i-1, 0], dh_para[i-1, 1], dh_para[i-1, 2], q[i-1])
            T_0_i = np.dot(T_0_i, T)

        z_i = T_0_i[0:3, 2]
        p_i = T_0_i[0:3, 3]
        r = spitze - p_i
        J[0:3, i] = np.cross(z_i, r) # J_translation
        J[3:6, i] = z_i              # J_rotation     

    # [v]               [J_trans ]
    # [ ]   = J * q' =  [        ] * q'
    # [w]               [J_rot   ] 
        
    # 6x1    6x6 6x1
    
    return J
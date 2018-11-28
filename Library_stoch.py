#!/usr/bin/python
# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

"""
Created on Sat Oct 21 18:48:45 2017

@author: mustang
"""
import time
import numpy as np
import math
from math import cos, sin, atan2
from numpy.linalg import inv
import numpy.linalg as la

PI = math.pi

def UserControl(k, v0):
    if k == ord('W'):
        v = v0 + 0.1
    elif k == ord('S'):
        v = v0 - 0.1
    else:
        v = v0

    if k == ord('A'):
        omega = 0.25
    elif k == ord('D'):
        omega = -0.25
    else:
        omega = 0

    if k == ord('E'):
        vs = 0.25
    elif k == ord('Q'):
        vs = -0.25
    else:
        vs = 0

    return [v, vs, omega]

# %% Rotation Matrix
def Rotation_matrix(roll, yaw, pitch):
    # Roll, yaw, pitch follow these sequences
    # The axis of rotations are:  roll along X axis, yaw along Z axis, pitch
    # along Y axis. This convention may differ in the convention of axis under
    # consideration

    Rr = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0,
                                                           sin(roll), cos(roll)]])
    Ry = np.array([[cos(yaw), 0, sin(yaw)], [0, 1, 0], [-sin(yaw), 0,
                                                        cos(yaw)]])
    Rp = np.array([[cos(pitch), -sin(pitch), 0], [sin(pitch),
                                                  cos(pitch), 0], [0, 0, 1]])

    R = np.dot(Rr, np.dot(Ry, Rp))
    return R


# %% Body Orientation

def body_orientation(Gb, PHI):
    # Details about the reference
    # Gb: contains the zero condition address of the nodes in 3X4 matrix. The
    # address is written as follows: front-left, front-right, hind-left,
    # hind-right.
    # PHI: [roll; pitch ; yaw];

    rb = np.zeros([3, 4])
    R = Rotation_matrix(PHI[0], PHI[2], PHI[1])

    for i in range(0, 4):
        rb[:, i] = np.dot(R, Gb[:, i])

    return rb

# %% ---------------------------------Leg position-------------------

def leg_positions(Gl, q, rB):
    # Abduction positions are from 1 to 4
    # Hip positions are from 5 to 8
    # Knee positions are from 9 to 12

    rl = np.zeros([3, 8])
    for i in range(0, 4):
        Rh = np.dot(Rotx(q[0, i]), Rotz(q[1, i]))
        Rk = Rotz(q[2, i])
        rl[:, i] = Rh.dot(Gl[:, i]) + rB[:, i]
        rl[:, i + 4] = Rk.dot(Gl[:, i + 4]) + rl[:, i]
    return rl


# %% --------------------------------Jacobian----------------------------------

def Jacobian_calculation(q, Gl, n):
    J = np.zeros([3, 3])

    if n == 0 or n == 3:
        f = -1
    else:
        f = -1
    Ra = Rotx(f * q[0, n])
    dRa = dRotx(f * q[0, n])
    Rh = Rotz(f * q[1, n])
    dRh = dRotz(f * q[1, n])
    Rk = Rotz(f * q[2, n])
    dRk = dRotz(f * q[2, n])

    # Jacobian

    J0 = np.dot(dRa, Rh).dot(Gl[:, n]) + np.dot(dRa, Rk).dot(Gl[:, n + 4])
    J1 = np.dot(Ra, dRh).dot(Gl[:, n])
    J2 = np.dot(Ra, dRk).dot(Gl[:, n + 4])

    J[:, 0] = J0
    J[:, 1] = J1
    J[:, 2] = J2
    return J

def Jacobian_calculation_2d(q, Gl):
    J = np.zeros([2, 2])
    Rh = Rotz2(q[0])
    dRh = dRotz2(q[0])
    Rk = Rotz2(q[1])
    dRk = dRotz2(q[1])

    # Jacobian

    J1 = dRh.dot(Gl[:, 0])
    J2 = dRk.dot(Gl[:, 1])

    J[:, 0] = J1
    J[:, 1] = J2
    return J


def Jacobian_Inv(q, lh, lk):
    JI = np.array([[sin(q[1])/lh, -cos(q[1])/lk],[-sin(q[0])/lh, cos(q[0])/lh]])
    if q[1] == q[0]:
       delta = 0.001
    else:
        delta = 0
    det_J = sin(q[1] -q[0] + delta)
    if abs(det_J) < 0.002:
        det_J = np.sign(det_J)*0.002
    Jinv = 1/det_J * JI
    return Jinv


# %% Rotation matrix about a single axis

def Rotx(x):
    R = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    return R


def Roty(x):
    R = np.array([[cos(x), 0, sin(x)], [0, 1, 0], [-sin(x), 0, cos(x)]])
    return R


def Rotz(x):
    R = np.array([[cos(x), -sin(x), 0], [sin(x), cos(x), 0], [0, 0, 1]])
    return R


def dRotx(x):
    R = np.array([[0, 0, 0], [0, -sin(x), -cos(x)], [0, cos(x),
                                                     -sin(x)]])
    return R


def dRoty(x):
    R = np.array([[-sin(x), 0, cos(x)], [0, 0, 0], [-cos(x), 0,
                                                    -sin(x)]])
    return R


def dRotz(x):
    R = np.array([[-sin(x), -cos(x), 0], [cos(x), -sin(x), 0], [0, 0,
                                                                0]])
    return R

def Rotz2(x):
    R = np.array([[cos(x), -sin(x)], [sin(x), cos(x)]])
    return R

def dRotz2(x):
    R = np.array([[-sin(x), -cos(x)], [cos(x), -sin(x)]])
    return R

# %% -----------------------Inverse Kinematics --------------------------

def Inverse_Kinamatics(Lh, Lk, r):
    # r = Target point for the 2 link system r: 2x1 [a, b]
    # Lh = initial hip orientation. Usually expressed as Lh: 2x1 [0, lh]
    # Lk = initial knee vector. Lk: 2x1 [0, lk]

    l3 = np.dot(r, r) ** 0.5
    l1 = Lh[1]
    l2 = Lk[1]

    zai = atan2(r[1], r[0]) + PI
    alpha = PI/2 + zai
    v1 = (l1 ** 2 + l2 ** 2 - l3 ** 2)
    v2 = (-2 * l1 * l2)

    cInv = v1/v2
    if cInv >1: cInv = 1
    elif cInv <-1: cInV = -1

    q2 = math.acos(cInv)
    #q1 = math.asin(l2 * sin(math.pi - q2) / l3) + alpha
    q1 = math.atan2(l2*sin(q2), l2*cos(q2) + l1) + zai
    return [q1, q2]

def ik_point2point(p0, p1, pb, nx, ny, nz, l1, l2):
    r = p1 - p0
    rb = pb - p0
    rn = np.array([np.dot(nx, rb), np.dot(ny, rb), np.dot(nz, rb)])
    rnv = np.array([np.dot(nx, r), np.dot(ny, r), np.dot(nz, r)])

    beta = math.atan2(rnv[0][0], abs(rnv[1][0]))  # Angular rotation

    l = np.linalg.norm(r, 2)  # total length
    Z = (l ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if Z > 1:
        Z = 1

    alpha = math.atan2(rn[2][0], abs(rn[1][0]))  # Abduction angle
    phi = math.acos(Z)
    theta = beta + math.asin(l2 * sin(PI - phi) / l)

    phi_global = phi + theta
    Q = np.array([-alpha, theta, phi_global])

    return Q


def ik_3d(p0, p1, l1, l2):
    # x, z on horizontal plane
    # y on along vertical
    #print "Inputs to ik_3d()"
    #print p0, p1, l1, l2
    r = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
    #print "r = p1 - p0 = ", r
    beta = math.atan2(r[0], abs(r[1]))  # Angular rotation
    #print "Calculated beta=  math.atan2(r[0][0], abs(r[1][0])) = ", beta

    l = np.linalg.norm(r, 2)  # total length
    Z = (l ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if Z > 1:
        Z = 1
    elif Z <-1:
        Z = -1
    alpha = math.atan2(r[2], abs(r[1]))  # Abduction angle

    #print "Abduction angle: alpha= math.atan2(r[2][0], abs(r[1][0])) = ", alpha*180/PI

    phi = math.acos(Z)
    theta = beta + math.asin(l2 * sin(PI - phi) / l)
    #print "Knee and hip angle: phi and theta = beta + math.asin(l2 * sin(PI - phi) / l) =  ", phi *180/PI, theta*180/PI

    phi_global = phi + theta - PI
    Q = np.array([-alpha, theta, phi_global])

    return Q

# %% --------------------------- Slope estimation ----------------------------------------

def slope_estimator(F, rl):
    alpha = 100
    F_th = 0.5
    C = np.zeros([4, 1])
    for i in range(0, 4):
        C[i] = 1 / (1 + np.exp(-0.00001 - alpha * (F[i] - F_th)))
    Cs = np.sum(C)

    if Cs == 0 or Cs == 1:
        roll_grd = 0
        pitch_grd = 0
    elif Cs == 2:
        j = 0
        r = np.zeros([3, 2])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1
        r1 = r[:, 1] - r[:, 0]
        r2 = np.array([0, 0, 1])

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])

        R = RodriRot(P1, P2)
        roll_grd = 0
        pitch_grd = np.arctan(R[0, 1] / R[0, 0])

    elif Cs == 3:
        j = 0
        r = np.zeros([3, 3])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1

        r1 = r[:, 0] - r[:, 2]
        r2 = r[:, 1] - r[:, 2]

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])
        R = np.array(RodriRot(P1, P2))
        roll_grd = np.arctan(-R[1, 2] / R[2, 2])
        pitch_grd = np.arctan(-R[0, 1] / R[0, 0])
    else:
        roll_grd = 0
        pitch_grd = 0

    return [roll_grd, -pitch_grd]
    # the negetive value indicates actual ground ground elevation from the base of the slope


def RodriRot(P1, P2):
    p1 = P1 / la.norm(P1, 2)
    p2 = P2 / la.norm(P2, 2)
    phi = np.arccos(np.dot(p1, p2))
    v = np.cross(p1, p2)
    vl = la.norm(v, 2)
    if vl == 0:
        w = v  # / la.norm(v, 2)
    else:
        w = v / la.norm(v, 2)
    W = np.array([[0, w[2], -w[1]], [-w[2], 0, w[0]], [w[1], -w[0], 0]])
    R = np.eye(3) + np.sin(phi) * W + (1 - np.cos(phi)) * np.dot(W, W)

    return R

#### ======================= Filter =================================================

# Low pas filter
def LowPassFilter(x0, dt, ud, xd, alpha):
    alpha = 1.25
    u  = ud + alpha*(xd - x0)
    x = u * dt + x0
    return x#!/usr/bin/python
# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

"""
Created on Sat Oct 21 18:48:45 2017

@author: mustang
"""
import time
import numpy as np
import math
from math import cos, sin, atan2
from numpy.linalg import inv
import numpy.linalg as la

PI = math.pi

def UserControl(k, v0):
    if k == ord('W'):
        v = v0 + 0.1
    elif k == ord('S'):
        v = v0 - 0.1
    else:
        v = v0

    if k == ord('A'):
        omega = 0.25
    elif k == ord('D'):
        omega = -0.25
    else:
        omega = 0

    if k == ord('E'):
        vs = 0.25
    elif k == ord('Q'):
        vs = -0.25
    else:
        vs = 0

    return [v, vs, omega]

# %% Rotation Matrix
def Rotation_matrix(roll, yaw, pitch):
    # Roll, yaw, pitch follow these sequences
    # The axis of rotations are:  roll along X axis, yaw along Z axis, pitch
    # along Y axis. This convention may differ in the convention of axis under
    # consideration

    Rr = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0,
                                                           sin(roll), cos(roll)]])
    Ry = np.array([[cos(yaw), 0, sin(yaw)], [0, 1, 0], [-sin(yaw), 0,
                                                        cos(yaw)]])
    Rp = np.array([[cos(pitch), -sin(pitch), 0], [sin(pitch),
                                                  cos(pitch), 0], [0, 0, 1]])

    R = np.dot(Rr, np.dot(Ry, Rp))
    return R


# %% Body Orientation

def body_orientation(Gb, PHI):
    # Details about the reference
    # Gb: contains the zero condition address of the nodes in 3X4 matrix. The
    # address is written as follows: front-left, front-right, hind-left,
    # hind-right.
    # PHI: [roll; pitch ; yaw];

    rb = np.zeros([3, 4])
    R = Rotation_matrix(PHI[0], PHI[2], PHI[1])

    for i in range(0, 4):
        rb[:, i] = np.dot(R, Gb[:, i])

    return rb

# %% ---------------------------------Leg position-------------------

def leg_positions(Gl, q, rB):
    # Abduction positions are from 1 to 4
    # Hip positions are from 5 to 8
    # Knee positions are from 9 to 12

    rl = np.zeros([3, 8])
    for i in range(0, 4):
        Rh = np.dot(Rotx(q[0, i]), Rotz(q[1, i]))
        Rk = Rotz(q[2, i])
        rl[:, i] = Rh.dot(Gl[:, i]) + rB[:, i]
        rl[:, i + 4] = Rk.dot(Gl[:, i + 4]) + rl[:, i]
    return rl


# %% --------------------------------Jacobian----------------------------------

def Jacobian_calculation(q, Gl, n):
    J = np.zeros([3, 3])

    if n == 0 or n == 3:
        f = -1
    else:
        f = -1
    Ra = Rotx(f * q[0, n])
    dRa = dRotx(f * q[0, n])
    Rh = Rotz(f * q[1, n])
    dRh = dRotz(f * q[1, n])
    Rk = Rotz(f * q[2, n])
    dRk = dRotz(f * q[2, n])

    # Jacobian

    J0 = np.dot(dRa, Rh).dot(Gl[:, n]) + np.dot(dRa, Rk).dot(Gl[:, n + 4])
    J1 = np.dot(Ra, dRh).dot(Gl[:, n])
    J2 = np.dot(Ra, dRk).dot(Gl[:, n + 4])

    J[:, 0] = J0
    J[:, 1] = J1
    J[:, 2] = J2
    return J

def Jacobian_calculation_2d(q, Gl):
    J = np.zeros([2, 2])
    Rh = Rotz2(q[0])
    dRh = dRotz2(q[0])
    Rk = Rotz2(q[1])
    dRk = dRotz2(q[1])

    # Jacobian

    J1 = dRh.dot(Gl[:, 0])
    J2 = dRk.dot(Gl[:, 1])

    J[:, 0] = J1
    J[:, 1] = J2
    return J


def Jacobian_Inv(q, lh, lk):
    JI = np.array([[sin(q[1])/lh, -cos(q[1])/lk],[-sin(q[0])/lh, cos(q[0])/lh]])
    if q[1] == q[0]:
       delta = 0.001
    else:
        delta = 0
    det_J = sin(q[1] -q[0] + delta)
    if abs(det_J) < 0.002:
        det_J = np.sign(det_J)*0.002
    Jinv = 1/det_J * JI
    return Jinv


# %% Rotation matrix about a single axis

def Rotx(x):
    R = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    return R


def Roty(x):
    R = np.array([[cos(x), 0, sin(x)], [0, 1, 0], [-sin(x), 0, cos(x)]])
    return R


def Rotz(x):
    R = np.array([[cos(x), -sin(x), 0], [sin(x), cos(x), 0], [0, 0, 1]])
    return R


def dRotx(x):
    R = np.array([[0, 0, 0], [0, -sin(x), -cos(x)], [0, cos(x),
                                                     -sin(x)]])
    return R


def dRoty(x):
    R = np.array([[-sin(x), 0, cos(x)], [0, 0, 0], [-cos(x), 0,
                                                    -sin(x)]])
    return R


def dRotz(x):
    R = np.array([[-sin(x), -cos(x), 0], [cos(x), -sin(x), 0], [0, 0,
                                                                0]])
    return R

def Rotz2(x):
    R = np.array([[cos(x), -sin(x)], [sin(x), cos(x)]])
    return R

def dRotz2(x):
    R = np.array([[-sin(x), -cos(x)], [cos(x), -sin(x)]])
    return R

# %% -----------------------Inverse Kinematics --------------------------

def Inverse_Kinamatics(Lh, Lk, r):
    # r = Target point for the 2 link system r: 2x1 [a, b]
    # Lh = initial hip orientation. Usually expressed as Lh: 2x1 [0, lh]
    # Lk = initial knee vector. Lk: 2x1 [0, lk]

    l3 = np.dot(r, r) ** 0.5
    l1 = Lh[1]
    l2 = Lk[1]

    zai = atan2(r[1], r[0]) + PI
    alpha = PI/2 + zai
    v1 = (l1 ** 2 + l2 ** 2 - l3 ** 2)
    v2 = (-2 * l1 * l2)

    cInv = v1/v2
    if cInv >1: cInv = 1
    elif cInv <-1: cInV = -1

    q2 = math.acos(cInv)
    #q1 = math.asin(l2 * sin(math.pi - q2) / l3) + alpha
    q1 = math.atan2(l2*sin(q2), l2*cos(q2) + l1) + zai
    return [q1, q2]

def ik_point2point(p0, p1, pb, nx, ny, nz, l1, l2):
    r = p1 - p0
    rb = pb - p0
    rn = np.array([np.dot(nx, rb), np.dot(ny, rb), np.dot(nz, rb)])
    rnv = np.array([np.dot(nx, r), np.dot(ny, r), np.dot(nz, r)])

    beta = math.atan2(rnv[0][0], abs(rnv[1][0]))  # Angular rotation

    l = np.linalg.norm(r, 2)  # total length
    Z = (l ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if Z > 1:
        Z = 1

    alpha = math.atan2(rn[2][0], abs(rn[1][0]))  # Abduction angle
    phi = math.acos(Z)
    theta = beta + math.asin(l2 * sin(PI - phi) / l)

    phi_global = phi + theta
    Q = np.array([-alpha, theta, phi_global])

    return Q


def ik_3d(p0, p1, l1, l2):
    # x, z on horizontal plane
    # y on along vertical
    #print "Inputs to ik_3d()"
    #print p0, p1, l1, l2
    r = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
    #print "r = p1 - p0 = ", r
    beta = math.atan2(r[0], abs(r[1]))  # Angular rotation
    #print "Calculated beta=  math.atan2(r[0][0], abs(r[1][0])) = ", beta

    l = np.linalg.norm(r, 2)  # total length
    Z = (l ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if Z > 1:
        Z = 1
    elif Z <-1:
        Z = -1
    alpha = math.atan2(r[2], abs(r[1]))  # Abduction angle

    #print "Abduction angle: alpha= math.atan2(r[2][0], abs(r[1][0])) = ", alpha*180/PI

    phi = math.acos(Z)
    theta = beta + math.asin(l2 * sin(PI - phi) / l)
    #print "Knee and hip angle: phi and theta = beta + math.asin(l2 * sin(PI - phi) / l) =  ", phi *180/PI, theta*180/PI

    phi_global = phi + theta - PI
    Q = np.array([-alpha, theta, phi_global])

    return Q

# %% --------------------------- Slope estimation ----------------------------------------

def slope_estimator(F, rl):
    alpha = 100
    F_th = 0.5
    C = np.zeros([4, 1])
    for i in range(0, 4):
        C[i] = 1 / (1 + np.exp(-0.00001 - alpha * (F[i] - F_th)))
    Cs = np.sum(C)

    if Cs == 0 or Cs == 1:
        roll_grd = 0
        pitch_grd = 0
    elif Cs == 2:
        j = 0
        r = np.zeros([3, 2])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1
        r1 = r[:, 1] - r[:, 0]
        r2 = np.array([0, 0, 1])

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])

        R = RodriRot(P1, P2)
        roll_grd = 0
        pitch_grd = np.arctan(R[0, 1] / R[0, 0])

    elif Cs == 3:
        j = 0
        r = np.zeros([3, 3])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1

        r1 = r[:, 0] - r[:, 2]
        r2 = r[:, 1] - r[:, 2]

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])
        R = np.array(RodriRot(P1, P2))
        roll_grd = np.arctan(-R[1, 2] / R[2, 2])
        pitch_grd = np.arctan(-R[0, 1] / R[0, 0])
    else:
        roll_grd = 0
        pitch_grd = 0

    return [roll_grd, -pitch_grd]
    # the negetive value indicates actual ground ground elevation from the base of the slope


def RodriRot(P1, P2):
    p1 = P1 / la.norm(P1, 2)
    p2 = P2 / la.norm(P2, 2)
    phi = np.arccos(np.dot(p1, p2))
    v = np.cross(p1, p2)
    vl = la.norm(v, 2)
    if vl == 0:
        w = v  # / la.norm(v, 2)
    else:
        w = v / la.norm(v, 2)
    W = np.array([[0, w[2], -w[1]], [-w[2], 0, w[0]], [w[1], -w[0], 0]])
    R = np.eye(3) + np.sin(phi) * W + (1 - np.cos(phi)) * np.dot(W, W)

    return R

#### ======================= Filter =================================================

# Low pas filter
def LowPassFilter(x0, dt, ud, xd, alpha):
    alpha = 1.25
    u  = ud + alpha*(xd - x0)
    x = u * dt + x0
    return x#!/usr/bin/python
# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

"""
Created on Sat Oct 21 18:48:45 2017

@author: mustang
"""
import time
import numpy as np
import math
from math import cos, sin, atan2
from numpy.linalg import inv
import numpy.linalg as la

PI = math.pi

def UserControl(k, v0):
    if k == ord('W'):
        v = v0 + 0.1
    elif k == ord('S'):
        v = v0 - 0.1
    else:
        v = v0

    if k == ord('A'):
        omega = 0.25
    elif k == ord('D'):
        omega = -0.25
    else:
        omega = 0

    if k == ord('E'):
        vs = 0.25
    elif k == ord('Q'):
        vs = -0.25
    else:
        vs = 0

    return [v, vs, omega]

# %% Rotation Matrix
def Rotation_matrix(roll, yaw, pitch):
    # Roll, yaw, pitch follow these sequences
    # The axis of rotations are:  roll along X axis, yaw along Z axis, pitch
    # along Y axis. This convention may differ in the convention of axis under
    # consideration

    Rr = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0,
                                                           sin(roll), cos(roll)]])
    Ry = np.array([[cos(yaw), 0, sin(yaw)], [0, 1, 0], [-sin(yaw), 0,
                                                        cos(yaw)]])
    Rp = np.array([[cos(pitch), -sin(pitch), 0], [sin(pitch),
                                                  cos(pitch), 0], [0, 0, 1]])

    R = np.dot(Rr, np.dot(Ry, Rp))
    return R


# %% Body Orientation

def body_orientation(Gb, PHI):
    # Details about the reference
    # Gb: contains the zero condition address of the nodes in 3X4 matrix. The
    # address is written as follows: front-left, front-right, hind-left,
    # hind-right.
    # PHI: [roll; pitch ; yaw];

    rb = np.zeros([3, 4])
    R = Rotation_matrix(PHI[0], PHI[2], PHI[1])

    for i in range(0, 4):
        rb[:, i] = np.dot(R, Gb[:, i])

    return rb

# %% ---------------------------------Leg position-------------------

def leg_positions(Gl, q, rB):
    # Abduction positions are from 1 to 4
    # Hip positions are from 5 to 8
    # Knee positions are from 9 to 12

    rl = np.zeros([3, 8])
    for i in range(0, 4):
        Rh = np.dot(Rotx(q[0, i]), Rotz(q[1, i]))
        Rk = Rotz(q[2, i])
        rl[:, i] = Rh.dot(Gl[:, i]) + rB[:, i]
        rl[:, i + 4] = Rk.dot(Gl[:, i + 4]) + rl[:, i]
    return rl


# %% --------------------------------Jacobian----------------------------------

def Jacobian_calculation(q, Gl, n):
    J = np.zeros([3, 3])

    if n == 0 or n == 3:
        f = -1
    else:
        f = -1
    Ra = Rotx(f * q[0, n])
    dRa = dRotx(f * q[0, n])
    Rh = Rotz(f * q[1, n])
    dRh = dRotz(f * q[1, n])
    Rk = Rotz(f * q[2, n])
    dRk = dRotz(f * q[2, n])

    # Jacobian

    J0 = np.dot(dRa, Rh).dot(Gl[:, n]) + np.dot(dRa, Rk).dot(Gl[:, n + 4])
    J1 = np.dot(Ra, dRh).dot(Gl[:, n])
    J2 = np.dot(Ra, dRk).dot(Gl[:, n + 4])

    J[:, 0] = J0
    J[:, 1] = J1
    J[:, 2] = J2
    return J

def Jacobian_calculation_2d(q, Gl):
    J = np.zeros([2, 2])
    Rh = Rotz2(q[0])
    dRh = dRotz2(q[0])
    Rk = Rotz2(q[1])
    dRk = dRotz2(q[1])

    # Jacobian

    J1 = dRh.dot(Gl[:, 0])
    J2 = dRk.dot(Gl[:, 1])

    J[:, 0] = J1
    J[:, 1] = J2
    return J


def Jacobian_Inv(q, lh, lk):
    JI = np.array([[sin(q[1])/lh, -cos(q[1])/lk],[-sin(q[0])/lh, cos(q[0])/lh]])
    if q[1] == q[0]:
       delta = 0.001
    else:
        delta = 0
    det_J = sin(q[1] -q[0] + delta)
    if abs(det_J) < 0.002:
        det_J = np.sign(det_J)*0.002
    Jinv = 1/det_J * JI
    return Jinv


# %% Rotation matrix about a single axis

def Rotx(x):
    R = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    return R


def Roty(x):
    R = np.array([[cos(x), 0, sin(x)], [0, 1, 0], [-sin(x), 0, cos(x)]])
    return R


def Rotz(x):
    R = np.array([[cos(x), -sin(x), 0], [sin(x), cos(x), 0], [0, 0, 1]])
    return R


def dRotx(x):
    R = np.array([[0, 0, 0], [0, -sin(x), -cos(x)], [0, cos(x),
                                                     -sin(x)]])
    return R


def dRoty(x):
    R = np.array([[-sin(x), 0, cos(x)], [0, 0, 0], [-cos(x), 0,
                                                    -sin(x)]])
    return R


def dRotz(x):
    R = np.array([[-sin(x), -cos(x), 0], [cos(x), -sin(x), 0], [0, 0,
                                                                0]])
    return R

def Rotz2(x):
    R = np.array([[cos(x), -sin(x)], [sin(x), cos(x)]])
    return R

def dRotz2(x):
    R = np.array([[-sin(x), -cos(x)], [cos(x), -sin(x)]])
    return R

# %% -----------------------Inverse Kinematics --------------------------

def Inverse_Kinamatics(Lh, Lk, r):
    # r = Target point for the 2 link system r: 2x1 [a, b]
    # Lh = initial hip orientation. Usually expressed as Lh: 2x1 [0, lh]
    # Lk = initial knee vector. Lk: 2x1 [0, lk]

    l3 = np.dot(r, r) ** 0.5
    l1 = Lh[1]
    l2 = Lk[1]

    zai = atan2(r[1], r[0]) + PI
    alpha = PI/2 + zai
    v1 = (l1 ** 2 + l2 ** 2 - l3 ** 2)
    v2 = (-2 * l1 * l2)

    cInv = v1/v2
    if cInv >1: cInv = 1
    elif cInv <-1: cInV = -1

    q2 = math.acos(cInv)
    #q1 = math.asin(l2 * sin(math.pi - q2) / l3) + alpha
    q1 = math.atan2(l2*sin(q2), l2*cos(q2) + l1) + zai
    return [q1, q2]

def ik_point2point(p0, p1, pb, nx, ny, nz, l1, l2):
    r = p1 - p0
    rb = pb - p0
    rn = np.array([np.dot(nx, rb), np.dot(ny, rb), np.dot(nz, rb)])
    rnv = np.array([np.dot(nx, r), np.dot(ny, r), np.dot(nz, r)])

    beta = math.atan2(rnv[0][0], abs(rnv[1][0]))  # Angular rotation

    l = np.linalg.norm(r, 2)  # total length
    Z = (l ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if Z > 1:
        Z = 1

    alpha = math.atan2(rn[2][0], abs(rn[1][0]))  # Abduction angle
    phi = math.acos(Z)
    theta = beta + math.asin(l2 * sin(PI - phi) / l)

    phi_global = phi + theta
    Q = np.array([-alpha, theta, phi_global])

    return Q


def ik_3d(p0, p1, l1, l2):
    # x, z on horizontal plane
    # y on along vertical
    #print "Inputs to ik_3d()"
    #print p0, p1, l1, l2
    r = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
    #print "r = p1 - p0 = ", r
    beta = math.atan2(r[0], abs(r[1]))  # Angular rotation
    #print "Calculated beta=  math.atan2(r[0][0], abs(r[1][0])) = ", beta

    l = np.linalg.norm(r, 2)  # total length
    Z = (l ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if Z > 1:
        Z = 1
    elif Z <-1:
        Z = -1
    alpha = math.atan2(r[2], abs(r[1]))  # Abduction angle

    #print "Abduction angle: alpha= math.atan2(r[2][0], abs(r[1][0])) = ", alpha*180/PI

    phi = math.acos(Z)
    theta = beta + math.asin(l2 * sin(PI - phi) / l)
    #print "Knee and hip angle: phi and theta = beta + math.asin(l2 * sin(PI - phi) / l) =  ", phi *180/PI, theta*180/PI

    phi_global = phi + theta - PI
    Q = np.array([-alpha, theta, phi_global])

    return Q

# %% --------------------------- Slope estimation ----------------------------------------

def slope_estimator(F, rl):
    alpha = 100
    F_th = 0.5
    C = np.zeros([4, 1])
    for i in range(0, 4):
        C[i] = 1 / (1 + np.exp(-0.00001 - alpha * (F[i] - F_th)))
    Cs = np.sum(C)

    if Cs == 0 or Cs == 1:
        roll_grd = 0
        pitch_grd = 0
    elif Cs == 2:
        j = 0
        r = np.zeros([3, 2])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1
        r1 = r[:, 1] - r[:, 0]
        r2 = np.array([0, 0, 1])

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])

        R = RodriRot(P1, P2)
        roll_grd = 0
        pitch_grd = np.arctan(R[0, 1] / R[0, 0])

    elif Cs == 3:
        j = 0
        r = np.zeros([3, 3])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1

        r1 = r[:, 0] - r[:, 2]
        r2 = r[:, 1] - r[:, 2]

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])
        R = np.array(RodriRot(P1, P2))
        roll_grd = np.arctan(-R[1, 2] / R[2, 2])
        pitch_grd = np.arctan(-R[0, 1] / R[0, 0])
    else:
        roll_grd = 0
        pitch_grd = 0

    return [roll_grd, -pitch_grd]
    # the negetive value indicates actual ground ground elevation from the base of the slope


def RodriRot(P1, P2):
    p1 = P1 / la.norm(P1, 2)
    p2 = P2 / la.norm(P2, 2)
    phi = np.arccos(np.dot(p1, p2))
    v = np.cross(p1, p2)
    vl = la.norm(v, 2)
    if vl == 0:
        w = v  # / la.norm(v, 2)
    else:
        w = v / la.norm(v, 2)
    W = np.array([[0, w[2], -w[1]], [-w[2], 0, w[0]], [w[1], -w[0], 0]])
    R = np.eye(3) + np.sin(phi) * W + (1 - np.cos(phi)) * np.dot(W, W)

    return R

#### ======================= Filter =================================================

# Low pas filter
def LowPassFilter(x0, dt, ud, xd, alpha):
    alpha = 1.25
    u  = ud + alpha*(xd - x0)
    x = u * dt + x0
    return x#!/usr/bin/python
# -*- coding: utf-8 -*-

# -*- coding: utf-8 -*-

"""
Created on Sat Oct 21 18:48:45 2017

@author: mustang
"""
import time
import numpy as np
import math
from math import cos, sin, atan2
from numpy.linalg import inv
import numpy.linalg as la

PI = math.pi

def UserControl(k, v0):
    if k == ord('W'):
        v = v0 + 0.1
    elif k == ord('S'):
        v = v0 - 0.1
    else:
        v = v0

    if k == ord('A'):
        omega = 0.25
    elif k == ord('D'):
        omega = -0.25
    else:
        omega = 0

    if k == ord('E'):
        vs = 0.25
    elif k == ord('Q'):
        vs = -0.25
    else:
        vs = 0

    return [v, vs, omega]

# %% Rotation Matrix
def Rotation_matrix(roll, yaw, pitch):
    # Roll, yaw, pitch follow these sequences
    # The axis of rotations are:  roll along X axis, yaw along Z axis, pitch
    # along Y axis. This convention may differ in the convention of axis under
    # consideration

    Rr = np.array([[1, 0, 0], [0, cos(roll), -sin(roll)], [0,
                                                           sin(roll), cos(roll)]])
    Ry = np.array([[cos(yaw), 0, sin(yaw)], [0, 1, 0], [-sin(yaw), 0,
                                                        cos(yaw)]])
    Rp = np.array([[cos(pitch), -sin(pitch), 0], [sin(pitch),
                                                  cos(pitch), 0], [0, 0, 1]])

    R = np.dot(Rr, np.dot(Ry, Rp))
    return R


# %% Body Orientation

def body_orientation(Gb, PHI):
    # Details about the reference
    # Gb: contains the zero condition address of the nodes in 3X4 matrix. The
    # address is written as follows: front-left, front-right, hind-left,
    # hind-right.
    # PHI: [roll; pitch ; yaw];

    rb = np.zeros([3, 4])
    R = Rotation_matrix(PHI[0], PHI[2], PHI[1])

    for i in range(0, 4):
        rb[:, i] = np.dot(R, Gb[:, i])

    return rb

# %% ---------------------------------Leg position-------------------

def leg_positions(Gl, q, rB):
    # Abduction positions are from 1 to 4
    # Hip positions are from 5 to 8
    # Knee positions are from 9 to 12

    rl = np.zeros([3, 8])
    for i in range(0, 4):
        Rh = np.dot(Rotx(q[0, i]), Rotz(q[1, i]))
        Rk = Rotz(q[2, i])
        rl[:, i] = Rh.dot(Gl[:, i]) + rB[:, i]
        rl[:, i + 4] = Rk.dot(Gl[:, i + 4]) + rl[:, i]
    return rl


# %% --------------------------------Jacobian----------------------------------

def Jacobian_calculation(q, Gl, n):
    J = np.zeros([3, 3])

    if n == 0 or n == 3:
        f = -1
    else:
        f = -1
    Ra = Rotx(f * q[0, n])
    dRa = dRotx(f * q[0, n])
    Rh = Rotz(f * q[1, n])
    dRh = dRotz(f * q[1, n])
    Rk = Rotz(f * q[2, n])
    dRk = dRotz(f * q[2, n])

    # Jacobian

    J0 = np.dot(dRa, Rh).dot(Gl[:, n]) + np.dot(dRa, Rk).dot(Gl[:, n + 4])
    J1 = np.dot(Ra, dRh).dot(Gl[:, n])
    J2 = np.dot(Ra, dRk).dot(Gl[:, n + 4])

    J[:, 0] = J0
    J[:, 1] = J1
    J[:, 2] = J2
    return J

def Jacobian_calculation_2d(q, Gl):
    J = np.zeros([2, 2])
    Rh = Rotz2(q[0])
    dRh = dRotz2(q[0])
    Rk = Rotz2(q[1])
    dRk = dRotz2(q[1])

    # Jacobian

    J1 = dRh.dot(Gl[:, 0])
    J2 = dRk.dot(Gl[:, 1])

    J[:, 0] = J1
    J[:, 1] = J2
    return J


def Jacobian_Inv(q, lh, lk):
    JI = np.array([[sin(q[1])/lh, -cos(q[1])/lk],[-sin(q[0])/lh, cos(q[0])/lh]])
    if q[1] == q[0]:
       delta = 0.001
    else:
        delta = 0
    det_J = sin(q[1] -q[0] + delta)
    if abs(det_J) < 0.002:
        det_J = np.sign(det_J)*0.002
    Jinv = 1/det_J * JI
    return Jinv


# %% Rotation matrix about a single axis

def Rotx(x):
    R = np.array([[1, 0, 0], [0, cos(x), -sin(x)], [0, sin(x), cos(x)]])
    return R


def Roty(x):
    R = np.array([[cos(x), 0, sin(x)], [0, 1, 0], [-sin(x), 0, cos(x)]])
    return R


def Rotz(x):
    R = np.array([[cos(x), -sin(x), 0], [sin(x), cos(x), 0], [0, 0, 1]])
    return R


def dRotx(x):
    R = np.array([[0, 0, 0], [0, -sin(x), -cos(x)], [0, cos(x),
                                                     -sin(x)]])
    return R


def dRoty(x):
    R = np.array([[-sin(x), 0, cos(x)], [0, 0, 0], [-cos(x), 0,
                                                    -sin(x)]])
    return R


def dRotz(x):
    R = np.array([[-sin(x), -cos(x), 0], [cos(x), -sin(x), 0], [0, 0,
                                                                0]])
    return R

def Rotz2(x):
    R = np.array([[cos(x), -sin(x)], [sin(x), cos(x)]])
    return R

def dRotz2(x):
    R = np.array([[-sin(x), -cos(x)], [cos(x), -sin(x)]])
    return R

# %% -----------------------Inverse Kinematics --------------------------

def Inverse_Kinamatics(Lh, Lk, r):
    # r = Target point for the 2 link system r: 2x1 [a, b]
    # Lh = initial hip orientation. Usually expressed as Lh: 2x1 [0, lh]
    # Lk = initial knee vector. Lk: 2x1 [0, lk]

    l3 = np.dot(r, r) ** 0.5
    l1 = Lh[1]
    l2 = Lk[1]

    zai = atan2(r[1], r[0]) + PI
    alpha = PI/2 + zai
    v1 = (l1 ** 2 + l2 ** 2 - l3 ** 2)
    v2 = (-2 * l1 * l2)

    cInv = v1/v2
    if cInv >1: cInv = 1
    elif cInv <-1: cInV = -1

    q2 = math.acos(cInv)
    #q1 = math.asin(l2 * sin(math.pi - q2) / l3) + alpha
    q1 = math.atan2(l2*sin(q2), l2*cos(q2) + l1) + zai
    return [q1, q2]

def ik_point2point(p0, p1, pb, nx, ny, nz, l1, l2):
    r = p1 - p0
    rb = pb - p0
    rn = np.array([np.dot(nx, rb), np.dot(ny, rb), np.dot(nz, rb)])
    rnv = np.array([np.dot(nx, r), np.dot(ny, r), np.dot(nz, r)])

    beta = math.atan2(rnv[0][0], abs(rnv[1][0]))  # Angular rotation

    l = np.linalg.norm(r, 2)  # total length
    Z = (l ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if Z > 1:
        Z = 1

    alpha = math.atan2(rn[2][0], abs(rn[1][0]))  # Abduction angle
    phi = math.acos(Z)
    theta = beta + math.asin(l2 * sin(PI - phi) / l)

    phi_global = phi + theta
    Q = np.array([-alpha, theta, phi_global])

    return Q


def ik_3d(p0, p1, l1, l2):
    # x, z on horizontal plane
    # y on along vertical
    #print "Inputs to ik_3d()"
    #print p0, p1, l1, l2
    r = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]]
    #print "r = p1 - p0 = ", r
    beta = math.atan2(r[0], abs(r[1]))  # Angular rotation
    #print "Calculated beta=  math.atan2(r[0][0], abs(r[1][0])) = ", beta

    l = np.linalg.norm(r, 2)  # total length
    Z = (l ** 2 - l1 ** 2 - l2 ** 2) / (2 * l1 * l2)
    if Z > 1:
        Z = 1
    elif Z <-1:
        Z = -1
    alpha = math.atan2(r[2], abs(r[1]))  # Abduction angle

    #print "Abduction angle: alpha= math.atan2(r[2][0], abs(r[1][0])) = ", alpha*180/PI

    phi = math.acos(Z)
    theta = beta + math.asin(l2 * sin(PI - phi) / l)
    #print "Knee and hip angle: phi and theta = beta + math.asin(l2 * sin(PI - phi) / l) =  ", phi *180/PI, theta*180/PI

    phi_global = phi + theta - PI
    Q = np.array([-alpha, theta, phi_global])

    return Q

# %% --------------------------- Slope estimation ----------------------------------------

def slope_estimator(F, rl):
    alpha = 100
    F_th = 0.5
    C = np.zeros([4, 1])
    for i in range(0, 4):
        C[i] = 1 / (1 + np.exp(-0.00001 - alpha * (F[i] - F_th)))
    Cs = np.sum(C)

    if Cs == 0 or Cs == 1:
        roll_grd = 0
        pitch_grd = 0
    elif Cs == 2:
        j = 0
        r = np.zeros([3, 2])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1
        r1 = r[:, 1] - r[:, 0]
        r2 = np.array([0, 0, 1])

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])

        R = RodriRot(P1, P2)
        roll_grd = 0
        pitch_grd = np.arctan(R[0, 1] / R[0, 0])

    elif Cs == 3:
        j = 0
        r = np.zeros([3, 3])
        for i in range(0, 4):
            if C[i] > 0.25:
                r[:, j] = rl[:, i + 4]
                j = j + 1

        r1 = r[:, 0] - r[:, 2]
        r2 = r[:, 1] - r[:, 2]

        P1 = np.cross(r1, r2)
        P2 = np.array([0, 1, 0])
        R = np.array(RodriRot(P1, P2))
        roll_grd = np.arctan(-R[1, 2] / R[2, 2])
        pitch_grd = np.arctan(-R[0, 1] / R[0, 0])
    else:
        roll_grd = 0
        pitch_grd = 0

    return [roll_grd, -pitch_grd]
    # the negetive value indicates actual ground ground elevation from the base of the slope


def RodriRot(P1, P2):
    p1 = P1 / la.norm(P1, 2)
    p2 = P2 / la.norm(P2, 2)
    phi = np.arccos(np.dot(p1, p2))
    v = np.cross(p1, p2)
    vl = la.norm(v, 2)
    if vl == 0:
        w = v  # / la.norm(v, 2)
    else:
        w = v / la.norm(v, 2)
    W = np.array([[0, w[2], -w[1]], [-w[2], 0, w[0]], [w[1], -w[0], 0]])
    R = np.eye(3) + np.sin(phi) * W + (1 - np.cos(phi)) * np.dot(W, W)

    return R

#### ======================= Filter =================================================

# Low pas filter
def LowPassFilter(x0, dt, ud, xd, alpha):
    alpha = 1.25
    u  = ud + alpha*(xd - x0)
    x = u * dt + x0
    return x

#!/usr/bin/env python3

from Frame import *
from scipy.optimize import minimize
from math import pi


# [n, ] -> scalar
def length(v):
    return np.linalg.norm(v, ord=2)


# [7, ] [7, ] -> scalar
def dis_l_l(l1, l2):
    p1, v1, r1 = l1[0:3], l1[3:6], l1[6]
    p2, v2, r2 = l2[0:3], l2[3:6], l2[6]
    cross = np.cross(v1, v2)
    return abs(np.sum((p1 - p2) * cross)) / length(cross) - r1 - r2


# [7, ] [7, ] -> scalar
def angle_l_l(l1, l2):
    v1 = l1[3:6]
    v2 = l2[3:6]
    return np.arccos(np.sum(v1*v2) / (length(v1) * length(v2)))


# [7, ] [3, ] -> scalar
def dis_l_t(l, t):
    p, v = l[0:3], l[3:6]
    cross = np.cross(t-p, v)
    return length(cross) / length(v)


# [4, 4] [7, ] -> [7, ]
def tran(r_6, l):
    c_new = np.zeros([7, ])
    p = np.ones([4, ])
    p[0:3] = l[0:3]
    v = l[3:6]
    f = Frame.from_r_3(r_6[0:3], r_6[3:6].reshape([3, 1]))
    c_new[0:3] = f.t_4_4.dot(p)[0:3].reshape([3, ])
    c_new[3:6] = f.r_3_3.dot(v).reshape([3, ])
    c_new[6] = l[6]
    return c_new


l = np.loadtxt("data/l.txt")
m = np.loadtxt("data/m.txt")
t = np.loadtxt("data/t.txt")

DISTANCE = 9.
ANGLE_L = 30. * pi / 180.
ANGLE_U = 150. * pi / 180.

eq_cons = {"type": "eq", "fun": lambda x: dis_l_l(tran(x, l[0]), m) - l[0, 6] - m[6] - DISTANCE}
ineq_cons = {"type": "ineq", "fun": lambda x: np.array([angle_l_l(tran(x, l[0]), m) - ANGLE_L
                                                          , - angle_l_l(tran(x, l[0]), m) + ANGLE_U])}

# get initial guess
l0_v = l[0][3:6]
m_v = m[3:6]

cross = np.cross(l0_v, m_v)
cross /= length(cross)
cross *= (DISTANCE - dis_l_l(l[0], m))
print(cross)
x0 = np.zeros([6, ])
x0[3:6] = -cross

print(dis_l_l(l[0], m) - DISTANCE)
print(dis_l_l(tran(x0, l[0]), m) - DISTANCE)


def cost(x):
    global l, t
    return dis_l_t(tran(x, l[1]), t) ** 2


res = minimize(cost, x0, method="SLSQP", constraints=[eq_cons, ineq_cons], options={"fto1": 1e-9, "disp": True})
print(res.x)

np.savetxt("result.txt", res.x)

#!/usr/bin/env python3

from Frame import *
from scipy.optimize import minimize
from math import pi
import sys
import yaml


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


with open("config/config.yml", 'r') as file:
    conf = yaml.safe_load(file.read())

# ====================
# m : scanned material
# b : built
# t : target
# ====================
task_index = conf["task_index"]
topo = np.loadtxt(conf["path_topo"])
path_result = conf["path_result"]
if topo[task_index][0] == -1:
    print("doesn't need to adapt")
    sys.exit(0)
elif topo[task_index][0] == 0:
    print("one constraint and one target")
    num_cons = 1
    m = np.array([np.loadtxt(conf["path_m"] % (task_index, topo[task_index][1])),
                  np.loadtxt(conf["path_m"] % (task_index, -1))])  # -1 : target
    b = np.loadtxt(conf["path_b"] % (task_index, topo[task_index][1]))
    t = np.loadtxt(conf["path_t"])[int(topo[task_index][4])]
elif topo[task_index][0] == 1:
    print("two constraint")
    num_cons = 2
    m = np.array([np.loadtxt(conf["path_m"] % (task_index, topo[task_index][1])),
                  np.loadtxt(conf["path_m"] % (task_index, topo[task_index][4]))])
    b = np.array([np.loadtxt(conf["path_b"] % (task_index, topo[task_index][1])),
                  np.loadtxt(conf["path_b"] % (task_index, topo[task_index][4]))])

# ===========
# constraints
# ===========
DISTANCE = conf["distance"]
ANGLE_L = conf["angle_l"] * pi / 180.
ANGLE_U = conf["angle_u"] * pi / 180.
mu = conf["mu"]

if num_cons == 1:
    eq_cons = {"type": "eq", "fun": lambda x: dis_l_l(tran(x, m[0]), b) - DISTANCE}
    ineq_cons = {"type": "ineq", "fun": lambda x: np.array([angle_l_l(tran(x, m[0]), b) - ANGLE_L,
                                                            - angle_l_l(tran(x, m[0]), b) + ANGLE_U])}
if num_cons == 2:
    eq_cons = {"type": "eq", "fun": lambda x: np.array([dis_l_l(tran(x, m[0]), b[0]) - DISTANCE,
                                                        dis_l_l(tran(x, m[1]), b[1]) - DISTANCE])}
    ineq_cons = {"type": "ineq", "fun": lambda x: np.array([angle_l_l(tran(x, m[0]), b[0]) - ANGLE_L,
                                                            - angle_l_l(tran(x, m[0]), b[0]) + ANGLE_U,
                                                            angle_l_l(tran(x, m[1]), b[1]) - ANGLE_L,
                                                            - angle_l_l(tran(x, m[1]), b[1]) + ANGLE_U])}

# get initial guess
# x0 = np.zeros([6, ])

# l0_v = l[0][3:6]
# m_v = m[0][3:6]
#
# cross = np.cross(l0_v, m_v)
# cross /= length(cross)
# cross *= (DISTANCE - dis_l_l(l[0], m[0]))
# print(cross)
x0 = np.zeros([6, ])
# x0[3:6] = -cross
#
# print(dis_l_l(l[0], m[0]) - DISTANCE)
# print(dis_l_l(tran(x0, l[0]), m[0]) - DISTANCE)


def cost(x):
    global m, t, mu
    if num_cons == 2:
        return length(x)
    if num_cons == 1:
        return dis_l_t(tran(x, m[1]), t) + mu * length(x)


res = minimize(cost, x0, method="SLSQP", constraints=[eq_cons, ineq_cons], options={"fto1": 1e-9, "disp": True})
print(res.x)

np.savetxt(path_result % task_index, res.x.reshape([1, 6]))

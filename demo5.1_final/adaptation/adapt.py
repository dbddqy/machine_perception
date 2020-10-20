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
    p1, v1, r1 = l1[1:4], l1[4:7], l1[0]
    p2, v2, r2 = l2[1:4], l2[4:7], l2[0]
    cross = np.cross(v1, v2)
    return abs(np.sum((p1 - p2) * cross)) / length(cross) - r1 - r2


# [7, ] [7, ] -> scalar
def angle_l_l(l1, l2):
    v1 = l1[4:7]
    v2 = l2[4:7]
    return np.arccos(np.sum(v1*v2) / (length(v1) * length(v2)))


# [7, ] [3, ] -> scalar
def dis_l_t(l, t):
    p, v = l[1:4], l[4:7]
    cross = np.cross(t-p, v)
    return length(cross) / length(v)


# [4, 4] [7, ] -> [7, ]
def tran(r_6, l):
    c_new = np.zeros([7, ])
    p = np.ones([4, ])
    p[0:3] = l[1:4]
    v = l[4:7]
    f = Frame.from_r_3(r_6[0:3], r_6[3:6].reshape([3, 1]))
    c_new[1:4] = f.t_4_4.dot(p)[0:3].reshape([3, ])
    c_new[4:7] = f.r_3_3.dot(v).reshape([3, ])
    c_new[0] = l[0]
    return c_new


# [3, ] [3, ] -> [4, 4]
def frame_from_normal(p, v):
    t = np.eye(4)
    # z axis
    z_temp = v / length(v)
    t[0:3, 2] = z_temp
    # x axis
    if abs(v[0]) < 1e-2 and abs(v[1]) < 1e-2:
        x_temp = np.cross(v, np.array([0., 0., 1.]))
        x_temp /= length(x_temp)
    else:
        x_temp = np.cross(v, np.array([0., 1., 0.]))
        x_temp /= length(x_temp)
    t[0:3, 0] = x_temp
    # y axis
    t[0:3, 1] = np.cross(z_temp, x_temp)
    # translation
    t[0:3, 3] = p
    return t


# [7, ] [7, ] -> [6, ]
def align_l_l(l1, l2):
    t1 = frame_from_normal(l1[1:4], l1[4:7])
    t2 = frame_from_normal(l2[1:4], l2[4:7])
    t21 = Frame(t2.dot(np.linalg.inv(t1)))
    result = np.zeros([6, ])
    result[0:3] = t21.r_3.flatten()
    result[3:6] = t21.t_3_1.flatten()
    return result


with open("config/config.yml", 'r') as file:
    conf = yaml.safe_load(file.read())

# ====================
# m : scanned material
# b : built
# t : target
# ====================
task_index = conf["task_index"]
# is_fliped = conf["is_fliped"]
topo = np.loadtxt(conf["path_topo"])
pole = np.loadtxt(conf["path_design"])[task_index]
path_result = conf["path_result"]
path_result_pre = conf["path_result_pre"]
path_result_target = conf["path_result_target"]
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
m_init = np.zeros([7, ])
m_init[0] = pole[0]
m_init[1:4] = pole[1:4] + topo[task_index][2] * pole[4:7] / length(pole[4:7])
m_init[4:7] = pole[4:7] / length(pole[4:7]) * 0.04
pre_tran = align_l_l(m[0], m_init)
m0 = tran(pre_tran, m[0])
m1 = tran(pre_tran, m[1])

# ===========
# constraints
# ===========
DISTANCE = conf["distance"]
ANGLE_L = conf["angle_l"] * pi / 180.
ANGLE_U = conf["angle_u"] * pi / 180.
mu = conf["mu"]

if num_cons == 1:
    eq_cons = {"type": "eq", "fun": lambda x: dis_l_l(tran(x, m0), b) - DISTANCE}
    ineq_cons = {"type": "ineq", "fun": lambda x: np.array([angle_l_l(tran(x, m0), b) - ANGLE_L,
                                                            - angle_l_l(tran(x, m0), b) + ANGLE_U])}
if num_cons == 2:
    eq_cons = {"type": "eq", "fun": lambda x: np.array([dis_l_l(tran(x, m0), b[0]) - DISTANCE,
                                                        dis_l_l(tran(x, m1), b[1]) - DISTANCE])}
    ineq_cons = {"type": "ineq", "fun": lambda x: np.array([angle_l_l(tran(x, m0), b[0]) - ANGLE_L,
                                                            - angle_l_l(tran(x, m0), b[0]) + ANGLE_U,
                                                            angle_l_l(tran(x, m1), b[1]) - ANGLE_L,
                                                            - angle_l_l(tran(x, m1), b[1]) + ANGLE_U])}

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
        return dis_l_t(tran(x, m1), t) + mu * length(x)


res = minimize(cost, x0, method="SLSQP", constraints=[eq_cons, ineq_cons], options={"fto1": 1e-9, "disp": True})
print(res.x)

# print angle
print("angle: ")
print(angle_l_l(tran(res.x, m0), b)*180/pi)

np.savetxt(path_result_pre % task_index, pre_tran.reshape([1, 6]), fmt="%f")
np.savetxt(path_result % task_index, res.x.reshape([1, 6]), fmt="%f")
np.savetxt(path_result_target % task_index, tran(res.x, m1).reshape([1, 7]), fmt="%f")

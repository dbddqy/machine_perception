from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
import numpy as np


# input: T[4, 4]
# output: T[4, 4]
def inv(t_4_4):
    t_4_4_new = t_4_4.copy()
    t_4_4_new[0:3, 3:4] = -t_4_4[0:3, 0:3].T.dot(t_4_4[0:3, 3:4])
    t_4_4_new[0:3, 0:3] = t_4_4[0:3, 0:3].T
    return t_4_4_new


# input: q[n, 4]
# output: q[n, 4]
def swap_quats(quats):
    quats_new = quats.copy()
    for i in range(quats_new.shape[0]):
        temp = quats[i, 0]
        quats_new[i, 0] = quats_new[i, 1]
        quats_new[i, 1] = quats_new[i, 2]
        quats_new[i, 2] = quats_new[i, 3]
        quats_new[i, 3] = temp
    return quats_new


# input: q[4,], t[3,]
# output: T[4, 4]
def t_4_4__quat(quat, t):
    t_4_4 = np.eye(4)
    t_4_4[0:3, 0:3] = R.from_quat(quat).as_matrix()
    t_4_4[0:3, 3:4] = t.reshape([3, 1])
    return t_4_4


# input: q[n, 4], t[n, 3]
# output: list of T[4, 4]
def t_4_4s__quats(quats, ts):
    t_4_4s = []
    for i in range(quats.shape[0]):
        t_4_4s.append(t_4_4__quat(quats[i], ts[i]))
    return t_4_4s


# input: q[n, 4], t[n, 3]
# output: list of T[4, 4]
def inv_t_4_4s__quats(quats, ts):
    t_4_4s = []
    for i in range(quats.shape[0]):
        t_4_4s.append(inv(t_4_4__quat(quats[i], ts[i])))
    return t_4_4s


# input: rvec[3,], t[3,]
# output: T[4, 4]
def t_4_4__rvec(rvec, t):
    t_4_4 = np.eye(4)
    t_4_4[0:3, 0:3] = R.from_rotvec(rvec).as_matrix()
    t_4_4[0:3, 3:4] = t.reshape([3, 1])
    return t_4_4


# input: rvec[6,]
# output: T[4, 4]
def t_4_4__rvec6(rvec):
    return t_4_4__rvec(rvec[0:3], rvec[3:6])


# input: rvecs[n, 6]
# output: T[4, 4]
def t_4_4s__rvecs6(rvecs):
    t_4_4s = []
    for i in range(rvecs.shape[0]):
        t_4_4s.append(t_4_4__rvec6(rvecs[i]))
    return t_4_4s


# input: T[4, 4]
# output: [rvec, t] [6,]
def rvec6__t_4_4(t_4_4):
    vec = np.zeros([6, ])
    vec[0:3] = R.from_matrix(t_4_4[0:3, 0:3]).as_rotvec()
    vec[3:6] = t_4_4[0:3, 3:4].reshape([3, ])
    return vec


# input: list of T[4, 4]
# output: [rvecs, ts] [n, 6]
def rvecs6__t_4_4s(t_4_4s):
    vecs = np.zeros([len(t_4_4s), 6])
    for i in range(len(t_4_4s)):
        vecs[i] = rvec6__t_4_4(t_4_4s[i])
    return vecs

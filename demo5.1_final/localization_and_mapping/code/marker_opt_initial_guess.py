#!/usr/bin/env python3
import numpy as np
import libs.lib_rs as rs
import libs.lib_frame as f
import cv2
import yaml


with open("config/config_marker.yml", 'r') as file:
    conf = yaml.safe_load(file.read())

n = conf["num_photos"]   # number of photos
m = conf["num_markers"]  # number of  markers
path_img = conf["path_img"] + "marker_%d.png"
path_result = conf["path_result"]
if conf["reorder"]:
    indices = conf["reorder_indices_list"]
else:
    indices = range(m)

d415 = rs.D415()
size = d415.marker_size

# ------------------------
# initial guesses w2k
# ------------------------

w2k = [np.eye(4)]*m
# w2k[0] = np.eye(4)
# w2k[1] = f.t_4_4__rvec(np.array([1.07888743e-02, -3.80953256e-02, -1.59157464e+00]),
#                        np.array([-9.08505140e-03, -4.23919083e-01, -1.34970647e-04]))
# w2k[2] = f.t_4_4__rvec(np.array([-2.29909471e-02, 1.96608912e-02, -1.56549568e+00]),
#                        np.array([6.33976050e-04, 3.89877312e-01, -5.58933275e-03]))

for i in range(1, m):
    for j in range(n):
        color = cv2.imread(path_img % j)
        corners, ids, _ = cv2.aruco.detectMarkers(color, d415.aruco_dict)
        # check if it contains i-th marker
        contain_i = False
        c2k_i, c2k_k = np.eye(4), np.eye(4)
        for k in range(len(corners)):
            if ids[k, 0] == indices[i]:
                contain_i = True
                c2k_i = rs.get_c2k(color, ids[k, 0], size, d415.C, d415.coeffs)
                break
        if not contain_i:
            continue
        # check if it contains marker with pose already known
        contain_know = False
        for k in range(len(corners)):
            if ids[k, 0] in indices[0:i]:
                contain_know = True
                c2k_k = rs.get_c2k(color, ids[k, 0], size, d415.C, d415.coeffs)
                w2k[i] = w2k[indices.index(ids[k, 0])].dot(f.inv(c2k_k)).dot(c2k_i)
                break
        if contain_know:
            break

np.save(path_result + "w2k_init", f.rvecs6__t_4_4s(w2k[1:]))  # exclude first one

# -------------------------
# for camera poses
# -------------------------

c2w = np.zeros([n, 6])
for i in range(n):
    color = cv2.imread(path_img % i)
    corners, ids, _ = cv2.aruco.detectMarkers(color, d415.aruco_dict)
    index = ids[0, 0]
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], size, d415.C, d415.coeffs)
    c2k = f.t_4_4__rvec(rvec.reshape([3, ]), tvec.reshape([3, ]))
    c2w[i] = f.rvec6__t_4_4(c2k.dot(f.inv(w2k[indices.index(index)])))
# print(c2w)
np.save(path_result + "c2w_init", c2w)

# -------------------------
# for marker relative poses
# -------------------------

# color = cv2.imread(path % 5)
# gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
# param = cv2.aruco.DetectorParameters_create()
# corners, ids, _ = cv2.aruco.detectMarkers(color, d415.aruco_dict)
# # corner refinement
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TermCriteria_COUNT, 40, 0.001)
# for i in range(len(corners)):
#     cv2.cornerSubPix(gray, corners[1], (3, 3), (-1, -1), criteria)
#
# cv2.aruco.drawDetectedMarkers(color, corners, ids)
# cv2.imshow("color", color)
# cv2.waitKey()
#
# c2k = []
# for i in range(2):
#     print(ids[i])
#     size = 0.2
#     rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size, d415.C_r, d415.coeffs_r)
#     # cv2.aruco.drawAxis(color, d415.C_r, d415.coeffs_r, rvec, tvec, 0.2)
#     # cv2.imshow("color", color)
#     # cv2.waitKey()
#     print("t")
#     print(tvec)
#     print("r")
#     print(rvec)
#     c2k.append(opt.t_4_4__rvec(rvec.reshape([3, ]), tvec.reshape([3, ])))
#
# print("================")
# k1 = opt.inv(c2k[1]).dot(c2k[0])
# print(opt.rvec6__t_4_4(k1))

d415.close()

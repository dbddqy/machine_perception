#!/usr/bin/env python3
import numpy as np
import libs.lib_rs as rs
import libs.lib_frame as f
import cv2
import yaml
from scipy.sparse import lil_matrix

# ------------------------
# solve: p = C * T(C->W) * T(W->K) * P(K)
# parameters to optimize:
# T(C->W) (6*n)
# T(W->K) (6*m)
# ------------------------

with open("config/config_marker.yml", 'r') as file:
    conf = yaml.safe_load(file.read())

n = conf["num_photos"]   # number of photos
m = conf["num_markers"]  # number of  markers
path_img = conf["path_img"] + "marker_%d.png"
path_result = conf["path_result"]
view_corner = conf["view_corner"]
corner_refinement = conf["corner_refinement"]
if conf["reorder"]:
    indices = conf["reorder_indices_list"]
else:
    indices = range(m)

d415 = rs.D415()
size = d415.marker_size


# ---------------------------------
# cost function: x: [c2w_rvec
#                    c2w_tvec
#                    ...(n times)
#                    w2k_rvec
#                    w2k_tvec
#                    ...(m-1 times)] # first one is identity
# ---------------------------------


def residual(x):
    global corners_all, ids_all, count_marker
    c2w = []
    for i in range(n):
        c2w.append(f.t_4_4__rvec(x[i * 6: i * 6 + 3], x[i * 6 + 3: i * 6 + 6]))
    w2k = []
    w2k.append(np.eye(4))
    for i in range(m-1):
        w2k.append(f.t_4_4__rvec(x[6 * n + i * 6: 6 * n + i * 6 + 3], x[6 * n + i * 6 + 3: 6 * n + i * 6 + 6]))
    # residual
    res = np.zeros([count_marker*4*2, ])
    marker_index = 0
    # res = []
    # i-th photo  j-th marker  k-th corner
    for i in range(len(corners_all)):
        corners, ids = corners_all[i], ids_all[i]
        for j in range(len(corners)):
            # if ids[j, 0] != i and ids[j, 0] != i+1:
            #     continue
            for k in range(4):
                corner = d415.C_ext.dot(c2w[i]).dot(w2k[indices.index(ids[j, 0])]).dot(rs.P(size, k))
                res[marker_index*8+k*2+0] = (corner[0][0] / corner[2][0] - corners[j][0, k][0])
                res[marker_index*8+k*2+1] = (corner[1][0] / corner[2][0] - corners[j][0, k][1])
            marker_index += 1
    return res


if view_corner:
    rs.window_setting("color")

# ------------------------
# load data
# ------------------------

corners_all, ids_all = [], []
count_marker = 0
for i in range(n):
    color = cv2.imread(path_img % i)
    if color is None:
        continue
    param = cv2.aruco.DetectorParameters_create()
    # param.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    param.cornerRefinementMethod = corner_refinement
    corners, ids, _ = cv2.aruco.detectMarkers(color, rs.aruco_dict(), parameters=param)
    if view_corner:
        cv2.aruco.drawDetectedMarkers(color, corners, ids)
        cv2.imshow("color", color)
        cv2.waitKey()
    corners_all.append(corners)
    ids_all.append(ids)
    count_marker += len(ids)

n = len(ids_all)
print("in %d images, totally %d markers" % (n, count_marker))

c2w_x0 = np.load(path_result + "c2w_init.npy").flatten()
w2k_x0 = np.load(path_result + "w2k_init.npy").flatten()
# w2k_4_4 = f.t_4_4s__rvecs6(w2k)
# c2w = []
# for i in range(len(ids_all)):
#     ids = ids_all[i]
#     corners = corners_all[i]
#     rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[0], 0.2, d415.C_r, d415.coeffs_r)
#     c2k = f.t_4_4__rvec(rvec.reshape([3, ]), tvec.reshape([3, ]))
#     c2w.append(f.rvec6__t_4_4(c2k.dot(f.inv(w2k_4_4[f.id_new(ids[0, 0])]))))
# c2w = np.array(c2w)
# c2w = c2w.flatten()
# w2k = w2k[1:].flatten()

x0 = np.hstack((c2w_x0, w2k_x0))

A = lil_matrix((count_marker * 8, n * 6 + (m - 1) * 6), dtype=int)
count_i_sum = 0
for i in range(n):
    count_i_th_photo = len(corners_all[i])
    for j in range(6):
        A[np.arange(count_i_sum*8, count_i_sum*8+count_i_th_photo*8), i*6+j] = 1
    for ids_i_th in ids_all[i][:, 0]:
        # index_i_th = indices.index(ids_i_th) - 1
        try:
            index_i_th = indices.index(ids_i_th) - 1
        except Exception:
            raise Exception("mis-detect in photo: %d" % i)
        if index_i_th != -1:
            for j in range(6):
                A[np.arange(count_i_sum*8, count_i_sum*8+count_i_th_photo*8), 6 * n + index_i_th * 6 + j] = 1
    count_i_sum += count_i_th_photo


ls = f.least_squares(residual, x0, jac_sparsity=A, verbose=2, x_scale='jac', ftol=1e-8)
# ls = f.least_squares(residual, x0, verbose=2, x_scale='jac', ftol=1e-4, method='trf')

print(ls.x[6 * n: 6 * n + 6])
print(ls.x[6 * n + 6: 6 * n + 12])

print("------------------")
print("final cost: %f" % ls.cost)
print("final optimality: %f" % ls.optimality)

final_w2k = []
final_w2k.append(np.eye(4).flatten())
for i in range(m-1):
    final_w2k.append(f.t_4_4__rvec6(ls.x[6*(n + i): 6 * (n + i + 1)]).flatten())
np.savetxt(path_result + "w2k.txt", final_w2k, fmt="%f")

for i in range(len(final_w2k)):
    np.savetxt(path_result + ("w2k_%d.txt" % i), final_w2k[i], fmt="%f")

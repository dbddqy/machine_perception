#!/usr/bin/env python3
import cv2
import libs.lib_rs as rs

# generation
dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

for i in range(250):
    image = cv2.aruco.drawMarker(dic, i, 2000)
    cv2.imwrite(("../markers_6/%d.png" % i), image)

# test detect
# cv2.namedWindow("color", cv2.WINDOW_NORMAL)
# cv2.resizeWindow("color", 960, 540)
#
# d415 = rs.D415()
#
# while cv2.waitKey(50) != ord('q'):
#     corners, ids, color, color_drawn = d415.detect_aruco()
#     cv2.imshow("color", color_drawn)
#
# d415.close()

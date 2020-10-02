import pyrealsense2 as rs
import libs.lib_frame as f
import cv2
import numpy as np
import yaml


class D415:
    def __init__(self, config_path="config/config.yml"):
        self.is_connected = True
        try:
            self.pipeline = rs.pipeline()
            self.align = rs.align(rs.stream.color)
            config = rs.config()
            config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
            cfg = self.pipeline.start(config)
        except:
            print("camera not connected.")
            self.is_connected = False
        # set intrinsics
        with open(config_path, 'r') as file:
            conf = yaml.safe_load(file.read())
        # profile = cfg.get_stream(rs.stream.color)
        # intrinsics = profile.as_video_stream_profile().get_intrinsics()
        self.width = conf["width"]
        self.height = conf["height"]
        self.ppx = conf["ppx"]
        self.ppy = conf["ppy"]
        self.fx = conf["fx"]
        self.fy = conf["fy"]
        self.coeffs = np.asarray(conf["coeffs"], dtype=np.float32).reshape([5, 1])
        self.C = np.array([[self.fx, 0., self.ppx],
                           [0., self.fy, self.ppy],
                           [0., 0., 1.]], dtype=np.float32)
        self.C_ext = np.array([[self.fx, 0., self.ppx, 0.],
                               [0., self.fy, self.ppy, 0.],
                               [0., 0., 1., 0.]], dtype=np.float32)

        # aruco related
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.marker_size = conf["marker_size"]

    # ==================================
    # 2d functionality
    # ==================================
    def get_frames(self):
        if not self.is_connected:
            raise Exception("camera not connected.")
        # align color frame to depth frame
        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        color_rs = frames.get_color_frame()
        depth_rs = frames.get_depth_frame()

        color_mat = np.asanyarray(color_rs.as_frame().get_data())
        depth_mat = np.asanyarray(depth_rs.as_frame().get_data())
        return color_mat, depth_mat

    def get_frame_color(self):
        if not self.is_connected:
            raise Exception("camera not connected.")
        frames = self.pipeline.wait_for_frames()
        color_rs = frames.get_color_frame()
        color_mat = np.asanyarray(color_rs.as_frame().get_data())
        return color_mat

    def detect_aruco(self):
        color = self.get_frame_color()
        color_drawn = color.copy()
        corners, ids, _ = cv2.aruco.detectMarkers(color_drawn, self.aruco_dict)
        cv2.aruco.drawDetectedMarkers(color_drawn, corners, ids)
        for i in range(len(corners)):
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], self.marker_size, self.C, self.coeffs)
            cv2.aruco.drawAxis(color_drawn, self.C, self.coeffs, rvec, tvec, self.marker_size)
        return corners, ids, color, color_drawn

    # ==================================
    # 3d functionality
    # ==================================
    def construct_point(self, u, v, d):
        point = np.zeros([3, ])
        point[0] = (u - self.ppx_r) * d / self.fx_r
        point[1] = (v - self.ppy_r) * d / self.fy_r
        point[2] = d
        return point

    def close(self):
        self.pipeline.stop()

# utility functions


def window_setting(config):
    if config == "color":
        cv2.namedWindow("color", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("color", 960, 540)
    elif config == "color+depth":
        cv2.namedWindow("color", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("color", 960, 540)
        cv2.namedWindow("depth", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("depth", 960, 540)


def aruco_dict():
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)


def coeffs():
    return np.array([0., 0., 0., 0., 0.]).reshape([5, 1])


def P(size, index):
    if index == 0:
        return np.array([[-0.5*size], [0.5*size], [0.], [1.]])
    if index == 1:
        return np.array([[0.5*size], [0.5*size], [0.], [1.]])
    if index == 2:
        return np.array([[0.5*size], [-0.5*size], [0.], [1.]])
    if index == 3:
        return np.array([[-0.5*size], [-0.5*size], [0.], [1.]])


def get_c2k(color, index, size, C, coeffs):
    param = cv2.aruco.DetectorParameters_create()
    param.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    corners, ids, _ = cv2.aruco.detectMarkers(color, aruco_dict(), parameters=param)
    for i in range(ids.shape[0]):
        if ids[i, 0] == index:
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], size, C, coeffs)
            return f.t_4_4__rvec(rvec.reshape([3, ]), tvec.reshape([3, ]))
    return None

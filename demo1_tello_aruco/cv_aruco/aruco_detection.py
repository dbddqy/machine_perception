import cv2
import cv2.aruco as aruco
import numpy as np
import camera_config
from math import atan2


class Aruco_detector:
    def __init__(self):
        self.url = "http://192.168.3.22:8080/?action=stream"
        self.capture = cv2.VideoCapture(self.url)
        self.dictionary = aruco.custom_dictionary(10, 6)
        self.pose_world = Pose()
        self.pose_tello = Pose()
        self.pose_world_to_tello = Pose()
        self.marker_size_world = 0.04
        self.marker_size_tello = 0.02
        self.image = np.zeros((1, 1), dtype=np.float32)
        self.is_detected = False

    def detect(self):
        self.is_detected = False
        ret, frame = self.capture.read()
        grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # detection
        corners, ids, rejectedImgPoints = aruco.detectMarkers(grey, self.dictionary)
        if ids is not None:
            if len(ids) == 2:
                self.is_detected = True
        if self.is_detected:
            for index in range(len(ids)):
                if ids[index][0] == 0:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index], self.marker_size_world,
                                                                    camera_config.camera_matrix,
                                                                    camera_config.distortion)
                    self.pose_world = Pose(cv2.Rodrigues(rvec[0])[0], tvec[0].T)
                    # draw axis for the aruco markers
                    aruco.drawAxis(grey, camera_config.camera_matrix, camera_config.distortion, rvec, tvec,
                                   self.marker_size_world)
                if ids[index][0] == 1:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index], self.marker_size_tello,
                                                                    camera_config.camera_matrix,
                                                                    camera_config.distortion)
                    self.pose_tello = Pose(cv2.Rodrigues(rvec[0])[0], tvec[0].T)
                    # draw axis for the aruco markers
                    aruco.drawAxis(grey, camera_config.camera_matrix, camera_config.distortion, rvec, tvec,
                                   self.marker_size_tello)
            self.pose_world_to_tello = Pose(np.dot(self.pose_world.R.T, self.pose_tello.R),
                                            np.dot(self.pose_world.R.T, self.pose_tello.t - self.pose_world.t))
        self.image = grey

    def release(self):
        self.capture.release()
        cv2.destroyAllWindows()


class Pose:
    def __init__(self, R=np.eye(3, dtype=np.float32), t=np.zeros((3, 1), dtype=np.float32)):
        self.R = R
        self.t = t

    @property
    def x(self):
        return self.t[0][0]

    @property
    def y(self):
        return self.t[1][0]

    @property
    def z(self):
        return self.t[2][0]

    @property
    def yaw(self):
        return atan2(self.R[1][0], self.R[1][1])




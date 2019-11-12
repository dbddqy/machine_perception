import cv2
import cv2.aruco as aruco
import camera_config_tello
from Pose import *


class Aruco_detector_tello:
    def __init__(self):
        self.dictionary = aruco.custom_dictionary(10, 6)
        self.pose_world = Pose()
        # self.pose_tello = Pose()
        # self.pose_world_to_tello = Pose()
        self.marker_size_world = 0.2
        # self.marker_size_tello = 0.02
        self.image_out = np.zeros((1, 1), dtype=np.float32)
        self.is_detected = False

    def detect(self, image):
        self.is_detected = False

        # detection
        corners, ids, rejectedImgPoints = aruco.detectMarkers(image, self.dictionary)
        if ids is not None:
            if len(ids) == 1:
                self.is_detected = True
        if self.is_detected:
            for index in range(len(ids)):
                if ids[index][0] == 2:
                    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index], self.marker_size_world,
                                                                    camera_config_tello.camera_matrix,
                                                                    camera_config_tello.distortion)
                    self.pose_world = Pose(cv2.Rodrigues(rvec[0])[0], tvec[0].T)
                    # transform
                    mat_t = np.dot(Pose.rotational_x(-pi*0.5).T, self.pose_world.R.T)
                    self.pose_world.R = np.dot(mat_t, Pose.rotational_x(pi*0.5))
                    self.pose_world.t = -np.dot(mat_t, self.pose_world.t)
                    # draw axis for the aruco markers
                    aruco.drawAxis(image, camera_config_tello.camera_matrix, camera_config_tello.distortion, rvec, tvec,
                                   self.marker_size_world)
                # if ids[index][0] == 1:
                #     rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[index], self.marker_size_tello,
                #                                                     camera_config_py.camera_matrix,
                #                                                     camera_config_py.distortion)
                #     self.pose_tello = Pose(cv2.Rodrigues(rvec[0])[0], tvec[0].T)
                #     # draw axis for the aruco markers
                #     aruco.drawAxis(grey, camera_config_py.camera_matrix, camera_config_py.distortion, rvec, tvec,
                #                    self.marker_size_tello)
            # self.pose_world_to_tello = Pose(np.dot(self.pose_world.R.T, self.pose_tello.R),
            #                                 np.dot(self.pose_world.R.T, self.pose_tello.t - self.pose_world.t))
        self.image_out = image

    def get_pose_world(self):
        return [self.pose_world.x*100., self.pose_world.y*100., self.pose_world.z*100., self.pose_world.yaw]

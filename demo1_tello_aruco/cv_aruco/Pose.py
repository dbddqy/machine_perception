import numpy as np
from math import atan2, sin, cos, pi


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

    @staticmethod
    def rotational_x(theta):
        return np.array([[1., 0., 0.],
                         [0., cos(theta), -sin(theta)],
                         [0., sin(theta), cos(theta)]], dtype=np.float32)

    @staticmethod
    def rotational_y(theta):
        return np.array([[cos(theta), 0., sin(theta)],
                         [0., 1., 0.],
                         [-sin(theta), 0., cos(theta)]], dtype=np.float32)

    @staticmethod
    def rotational_z(theta):
        return np.array([[cos(theta), -sin(theta), 0.],
                         [sin(theta), cos(theta), 0.],
                         [0., 0., 1.]], dtype=np.float32)

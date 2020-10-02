//
// Created by yue on 23.09.20.
//

#ifndef DETECTION_LIB_CAMERA_HPP
#define DETECTION_LIB_CAMERA_HPP

#include <Eigen/Core>
#include <Eigen/Dense>  // for inverse
#include <opencv2/opencv.hpp>

namespace CameraConfig {
    static const double fx = 1382.23;
    static const double fy = 1379.46;
    static const double ppx = 953.567;
    static const double ppy = 532.635;
    static const Eigen::Matrix3d M = [] {
        Eigen::Matrix3d temp;
        temp << fx, 0., ppx,
                0., fy, ppy,
                0., 0., 1.;
        return temp;
    }();
    static const Eigen::Matrix3d MInv = M.inverse();
    static const Eigen::VectorXd distortion = [] {
        Eigen::VectorXd temp(5);
        temp << 0.0, 0.0, 0.0, 0.0, 0.0;
        return temp;
    }();
    static const cv::Mat M_Mat= (cv::Mat_<double>(3, 3)
            << fx, 0., ppx,
               0., fy, ppy,
               0., 0., 1.);
    static const cv::Mat distortion_Mat = (cv::Mat_<double >(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);
}

#endif //DETECTION_LIB_CAMERA_HPP

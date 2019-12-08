//
// Created by yue on 06.12.19.
//

#ifndef POSE_HPP
#define POSE_HPP

#include <opencv2/opencv.hpp>

using namespace cv;

class Pose {
public:
    Mat m;
    Pose(Point3d origin, Vec3d rotationVec);
    Pose(Point3d origin, Vec3d unitZ, Vec3d unitX);
    Mat R();
    Mat origin();
    Mat xAxis();
    Mat yAxis();
    Mat zAxis();
};

#endif //POSE_HPP

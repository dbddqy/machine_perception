#ifndef LIB_H
#define LIB_H

#include <string.h>
#include <iostream>
#include <stdlib.h>
#include <boost/format.hpp>  // for formating strings

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;

class CameraConfig {
public:
    Mat M, distortion, MExt, MInv, c2w, w2c;
    double fx, fy, cx, cy;
    CameraConfig();
};

CameraConfig C;

bool checkState(int f_state);

double **readData(string path, int row, int col);

int findIndex(vector<int> indices, int index);

Eigen::Vector4d getCorner(int index, double size);

Mat readPose();

double dis(Vec3d p1, Vec3d p2);

double distance2cylinder(double *cylinder_param, Vec3d pt, double start_param, double end_param);

#endif //LIB_H
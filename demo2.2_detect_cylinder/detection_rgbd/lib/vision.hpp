//
// Created by yue on 31.01.20.
//

#ifndef VISION_HPP
#define VISION_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include <stack>
#include <pose.hpp>

using namespace cv;
using namespace std;

class CameraConfig {
public:
    Mat M;
    Mat distortion;
    Mat MInv;
    double fx, fy, cx, cy;

    CameraConfig();
};

CameraConfig C;

void detectAruco(Mat img, vector<pose> &poses, vector<vector<Point2f> > &markerCorners, vector<int> &markerIds);

void grow(cv::Mat& src, cv::Mat& mask, cv::Point seed, int threshold);

#endif //VISION_HPP

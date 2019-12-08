//
// Created by yue on 06.12.19.
//

#ifndef MATCHING_HPP
#define MATCHING_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <CameraConfig.hpp>
#include <Pose.cpp>
#include <math.h>
#include <vector>

using namespace std;
using namespace cv;

bool validate(vector<vector<Point> > contoursL, vector<vector<Point> > contoursR, vector<vector<Point3d> > &realPolylines);

Point3d getCenter(vector<Point3d> polyline);

Vec3d getNormal(vector<Point3d> polyline);

Pose getPose(vector<Point3d> polyline);

Mat drawPose(Mat img, Pose pose, double length); // length in mm

Point vec2point(Mat v);

Vec3d unitize(Vec3d v);

double getLength(Vec3d v);

#endif //MATCHING_HPP

//
// Created by yue on 06.12.19.
//

#ifndef MATCHING_HPP
#define MATCHING_HPP

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <CameraConfig.hpp>
#include <Pose.cpp>
#include <math.h>
#include <vector>
#include <rapidjson/document.h>

using namespace std;
using namespace cv;
using namespace rapidjson;

vector< vector<Point3d> > getLibPieces();
vector< vector<Point3d> > LIBPIECES = getLibPieces();

bool validate(vector<vector<Point> > contoursL, vector<vector<Point> > contoursR, vector<vector<Point3d> > &realPolylines);

int match(vector<Point3d> polyline, Pose pose);

Point3d getCenter(vector<Point3d> polyline);

Vec3d getNormal(vector<Point3d> polyline);

Pose getPose(vector<Point3d> polyline);

Mat drawPose(Mat img, Pose pose, int index, double length); // length in mm

void sortPolyline(vector<Point3d> &polyline);

Point vec2point(Mat v);

Point3d transformCamera2Piece(Point3d p, Pose piecePose);

Vec3d unitize(Vec3d v);

double distance(Point3d p1, Point3d p2);

double getLength(Vec3d v);

bool readFileToString(string file_name, string& fileData);

#endif //MATCHING_HPP

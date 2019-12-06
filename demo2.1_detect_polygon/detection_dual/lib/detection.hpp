//
// Created by yue on 06.12.19.
//

#ifndef DETECTION_H
#define DETECTION_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

vector<Point> findLargestContour(vector< vector<Point> > contours);

vector< vector<Point> > findLargeContours(vector< vector<Point> > contours, double minArea);

vector< vector<Point> > getContours(Mat img);

#endif //DETECTION_H

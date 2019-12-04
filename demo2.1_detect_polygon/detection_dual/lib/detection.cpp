//
// Created by yue on 03.12.19.
//

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

vector<Point> findLargestContour(vector< vector<Point> > contours) {
    double largestArea = 0.; int largestIndex = 0;
    for (int i = 0; i < contours.size(); ++i) {
        double area = contourArea(contours[i]);
        if (area > largestArea){
            largestArea = area;
            largestIndex = i;
        }
    }
    return contours[largestIndex];
}

vector< vector<Point> > findLargeContours(vector< vector<Point> > contours, double minArea) {
    vector< vector<Point> > outPutContours;
    for (int i = 0; i < contours.size(); ++i) {
        double area = contourArea(contours[i]);
        if (area > minArea)
            outPutContours.push_back(contours[i]);
    }
    return outPutContours;
}

vector< vector<Point> > getContours(Mat img) {
    /* processing image:
     *  1.convert to greyscale and blurring
     *  2.edge extraction
     *  3.blurring
     *  4.binarizing
     *  5.find contours
     *  6.find largest contour
     *  7.contour to polyline(corners get!) */
    Mat imgGrey, imgBlur, imgFilter2d;
    // 1.convert to greyscale and blurring
    cvtColor(img, imgGrey, CV_BGR2GRAY);
    GaussianBlur(imgGrey, imgBlur, Size(9, 9), 2);
//    blur(imgGrey, imgBlur, Size(7, 7));

    // 2.edge extraction
//    Mat kernel = (Mat_<char>(3, 3) << 0, 1, 0, 1, -4, 1, 0, 1, 0); // Laplacian filter
    Mat kernel = (Mat_<char>(3, 3) << -1, -1, -1, -1, 8, -1, -1, -1, -1); // Laplacian filter
//    Mat kernel = (Mat_<char>(3, 3) << -1, -2, -1, 0, 0, 0, 1, 2, 1); // Sobel filter x_dir
    filter2D(imgBlur, imgFilter2d, -1, kernel);
    // 3.blurring
    blur(imgFilter2d, imgFilter2d, Size(9, 9));
    // 4.binarizing
    threshold(imgFilter2d, imgFilter2d, 3, 50, THRESH_BINARY);
    // 5.find contours
    vector<vector<Point> > contours;
    findContours(imgFilter2d, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//    cout << contours.size() << endl;
    // 6.find largest contour
    vector< vector<Point> > largeContours = findLargeContours(contours, 500.);
//    cout << largeContours.size() << endl;
    // 7.contour to polyline(corners get!)
    for (int i = 0; i < largeContours.size(); ++i)
        approxPolyDP(largeContours[i], largeContours[i], 10, true);
//    for (int i = 0; i < largeContours.size(); ++i)
//        cout << largeContours[i] << endl;
//    drawContours(img, largeContours, -1, Scalar(0, 0, 255));
//
//    imshow("myImage", img);
//    imshow("filter2d", imgFilter2d);
//    waitKey(0);
    return largeContours;
}

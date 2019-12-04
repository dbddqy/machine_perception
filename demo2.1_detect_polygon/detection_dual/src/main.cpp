#include <iostream>
#include "main.h"
#include <opencv2/opencv.hpp>
#include <detection.cpp>
#include <cameraConfig.cpp>

using namespace cv;
using namespace std;

int main() {
    VideoCapture inputVideo("../../data/02.mov");
    // load camera parameters
    Mat R1, R2, P1, P2, Q, mapL1, mapL2, mapR1, mapR2;
    cameraConfig::getCameraMatrices(R1, R2, P1, P2, Q);
    cameraConfig::getMaps(mapL1, mapL2, mapR1, mapR2);
    double b = -cameraConfig::T.at<double>(0, 0);
    double f = P1.at<double>(0, 0);
    Mat cameraLInv = P1(Rect(0, 0, 3, 3)).inv();
    Mat cameraRInv = P2(Rect(0, 0, 3, 3)).inv();
    while (true) {
        Mat img;
        inputVideo >> img;
        if (img.empty()) break;
        Rect rectL(0, 0, 640, 480);
        Rect rectR(640, 0, 640, 480);
        Mat imgL = img(rectL), imgR = img(rectR);
        remap(imgL, imgL, mapL1, mapL2, CV_INTER_LINEAR);
        remap(imgR, imgR, mapR1, mapR2, CV_INTER_LINEAR);
        // left
        vector< vector<Point> > contoursL = getContours(imgL);
        drawContours(imgL, contoursL, -1, Scalar(0, 0, 255));
        // right
        vector< vector<Point> > contoursR = getContours(imgR);
        drawContours(imgR, contoursR, -1, Scalar(0, 0, 255));
        // matching
        if (contoursL.size() == contoursR.size()) {
            for (int i = 0; i < contoursL.size(); ++i) {
                if (contoursL[i].size() != contoursR[i].size()) break;
                cout << "contour left: " << contoursL[i] << endl;
                cout << "contour right: " << contoursR[i] << endl;
                for (int j = 0; j < contoursL[i].size(); ++j) {
                    double depth = b * f / (contoursL[i][j].x - contoursR[i][j].x);
                    cout << depth << endl;
                    // position L
                    Mat positionLPixel = (Mat_<double>(3, 1)
                            << contoursL[i][j].x*depth, contoursL[i][j].y*depth, depth);
                    Mat positionL = cameraLInv * positionLPixel;
                    cout << "positionL: " << positionL << endl;
                    // position R
                    Mat positionRPixel = (Mat_<double>(3, 1)
                            << contoursR[i][j].x*depth, contoursR[i][j].y*depth, depth);
                    Mat positionR = cameraRInv * positionRPixel;
                    cout << "positionR: " << positionR << endl;
                }
            }
        }
        // show images
        imshow("rawL", imgL);
        imshow("rawR", imgR);

        char key = (char) waitKey(1);
        if (key == 27)
            break;
    }
    return 0;
}

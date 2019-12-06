//
// Created by yue on 06.12.19.
//

#ifndef CAMERA_CONFIG_H
#define CAMERA_CONFIG_H

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;

class CameraConfig {
public:
    Mat leftCameraMatrix, leftDistortion, rightCameraMatrix, rightDistortion;
    Mat RVec, R, T;
    Size size;
    Mat R1, R2, P1, P2, Q, mapL1, mapL2, mapR1, mapR2, cameraLInv, cameraRInv;
    double b, f;

    CameraConfig();

    Mat getR();

    void getCameraMatrices(Mat &outR1, Mat &outR2, Mat &outP1, Mat &outP2, Mat &outQ);

    void getMaps(Mat &outMapL1, Mat &outMapL2, Mat &outMapR1, Mat &outMapR2);
};

CameraConfig c;

#endif //CAMERA_CONFIG_H

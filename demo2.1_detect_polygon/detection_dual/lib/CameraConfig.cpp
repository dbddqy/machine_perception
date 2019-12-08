//
// Created by yue on 04.12.19.
//

#include <CameraConfig.hpp>

CameraConfig::CameraConfig() {
    leftCameraMatrix = (Mat_<double>(3, 3)
            << 891.99940, 0., 326.93216
            , 0., 892.35459, 235.09397
            , 0., 0., 1.);
    leftDistortion = (Mat_<double>(1, 5)
            << -0.43721, 0.24738, -0.00196, 0.00004, 0.00000);
    rightCameraMatrix = (Mat_<double>(3, 3)
            << 886.52934, 0., 318.26183
            , 0., 886.13278, 238.68409
            , 0., 0., 1.);
    rightDistortion = (Mat_<double>(1, 5)
            << -0.41575, 0.07315, -0.00450, -0.00067, 0.00000);
    RVec = (Mat_<double>(1, 3)
            << -0.00271, -0.01687, 0.00032);
    T = (Mat_<double>(3, 1)
            << -60.27202, 0.36238, -0.39229);
    R = getR();
    size = Size(640, 480);
    getCameraMatrices(R1, R2, P1, P2, Q);
    getMaps(mapL1, mapL2, mapR1, mapR2);
    b = -T.at<double>(0, 0);
    f = P1.at<double>(0, 0);
    cameraL = P1(Rect(0, 0, 3, 3));
    cameraR = P2(Rect(0, 0, 3, 3));
    cameraLInv = cameraL.inv();
    cameraRInv = cameraR.inv();
}

Mat CameraConfig::getR() {
    Mat mat;
    Rodrigues(RVec, mat);
    return mat;
}

void CameraConfig::getCameraMatrices(Mat &outR1, Mat &outR2, Mat &outP1, Mat &outP2, Mat &outQ) {
    stereoRectify(leftCameraMatrix, leftDistortion, rightCameraMatrix, rightDistortion
                , size, R, T, outR1, outR2, outP1, outP2, outQ);
}

void CameraConfig::getMaps(Mat &outMapL1, Mat &outMapL2, Mat &outMapR1, Mat &outMapR2) {
    Mat R1, R2, P1, P2, Q;
    getCameraMatrices(R1, R2, P1, P2, Q);
    initUndistortRectifyMap(leftCameraMatrix, leftDistortion, R1, P1, size, CV_32FC1, outMapL1, outMapL2);
    initUndistortRectifyMap(rightCameraMatrix, rightDistortion, R2, P2, size, CV_32FC1, outMapR1, outMapR2);
}

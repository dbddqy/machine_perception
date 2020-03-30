//
// Created by yue on 24.03.20.
//

#include <iostream>
#include <vector>
#include <boost/format.hpp>  // for formating strings

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <librealsense2/rs.hpp>

using namespace std;
using namespace cv;

class CameraConfig {
public:
    Mat M;
    Mat distortion;
    Mat MInv;
    double fx, fy, cx, cy;

    CameraConfig() {
        M = (Mat_<double>(3, 3)
                << 1382.23, 0., 953.567
                , 0., 1379.46, 532.635
                , 0., 0., 1.);
        distortion = (Mat_<double >(1, 5)
                << 0.0, 0.0, 0.0, 0.0, 0.0);
        MInv = M.inv();
        fx = 1382.23; fy = 1379.46; cx = 953.567; cy = 532.635;
    };
};

CameraConfig C;

bool detectChessboard(Mat img, Mat &tranc2o, Point &center) {
    vector<Point2f> centers;
    bool pattern_was_found = findCirclesGrid(img, cv::Size(2, 13), centers, CALIB_CB_ASYMMETRIC_GRID);
    if (!pattern_was_found)
        return false;
    cv::drawChessboardCorners(img, cv::Size(2, 13), Mat(centers), pattern_was_found);
    vector<Point3f> objectPoints;

    for (int i = 0; i < 13; ++i)
        for (int j = 0; j < 2; ++j)
            objectPoints.push_back(Point3d(i*0.02, j*0.04+(i%2)*0.02, 0.0));
    Mat rvec, tvec;
    Mat matCorneres = Mat(centers).reshape(1);
    Mat matObjectPoints = Mat(objectPoints).reshape(1);
    solvePnP(matObjectPoints, matCorneres, C.M, C.distortion, rvec, tvec);

    cv::aruco::drawAxis(img, C.M, C.distortion, rvec, tvec, 0.04);
    center = centers[0];
    Mat r;
    Rodrigues(rvec, r);
    hconcat(r, tvec, tranc2o);
    Mat down = (Mat_<double>(1, 4) << 0., 0., 0., 1.);
    vconcat(tranc2o, down, tranc2o);
    return true;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R) {
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    return  norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R) {
    assert(isRotationMatrix(R));
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    float x, y, z;
    if (!singular) {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    } else {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

int main(int argc, char **argv) {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);

    // start
    while (waitKey(1) != 'q') {
        rs2::frameset frameset = pipe.wait_for_frames();
        frameset = align_to_color.process(frameset);

        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();

        Mat matColor(Size(1920, 1080), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        Mat matDepth(Size(1920, 1080), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);

        Mat tran_c2o;
        Point center;

        if (detectChessboard(matColor, tran_c2o, center)) {
            Mat tran_02c = tran_c2o.inv();
            Mat R = tran_02c(Rect(0, 0, 3, 3));
            Vec3f zyxAngles = rotationMatrixToEulerAngles(R) / M_PI * 180.0;
            boost::format fmt("Eular angles(ZYX): X: %f Y: %f Z: %f");
            putText(matColor, (fmt%zyxAngles[0]%zyxAngles[1]%zyxAngles[2]).str(),
                    Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1.2);
        }
        imshow("circle_qboard", matColor);
    }
    return 0;
}
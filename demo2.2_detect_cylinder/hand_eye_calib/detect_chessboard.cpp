//
// Created by yue on 19.02.20.
//

#include <iostream>
#include <vector>

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
    vector<Point2f> corners;

    bool pattern_was_found = cv::findChessboardCorners(img, cv::Size(9, 6), corners);
    if (!pattern_was_found)
        return false;
    cv::drawChessboardCorners(img, cv::Size(9, 6), corners, pattern_was_found);
    vector<Point3f> objectPoints;

    for (int i = 0; i < 6; ++i)
        for (int j = 0; j < 9; ++j)
            objectPoints.push_back(Point3d(i*0.025, j*0.025, 0.0));
    Mat rvec, tvec;
    Mat matCorneres = Mat(corners).reshape(1);
    Mat matObjectPoints = Mat(objectPoints).reshape(1);
    solvePnP(matObjectPoints, matCorneres, C.M, C.distortion, rvec, tvec);

    cv::aruco::drawAxis(img, C.M, C.distortion, rvec, tvec, 0.125);
    center = corners[0];
    Mat r;
    Rodrigues(rvec, r);
//    Mat t = (Mat_<double>(3, 1) << tvec[0]*1000, tvec[1]*1000, tvec[2]*1000);
    hconcat(r, tvec, tranc2o);
    Mat down = (Mat_<double>(1, 4) << 0., 0., 0., 1.);
    vconcat(tranc2o, down, tranc2o);
    return true;
}

int main(int argc, char **argv) {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);

    int num_parameters_saved = 0;
    cout << "press 's' to print marker pose." << endl;
    // start
    while (waitKey(1) != 'q') {
        rs2::frameset frameset = pipe.wait_for_frames();
        frameset = align_to_color.process(frameset);

        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();

        Mat matColor(Size(1920, 1080), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        Mat matDepth(Size(1920, 1080), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);

        Mat tranc2o;
        Point center;

        if (detectChessboard(matColor, tranc2o, center))
            if (waitKey(1) == 's') {
                cout << "depth: " << matDepth.at<int16_t>(center) << endl;
                cout << "parameters: " << num_parameters_saved << endl;
                for (int i = 0; i < 4; ++i)
                    for (int j = 0; j < 4; ++j)
                        cout << tranc2o.at<double>(i, j) << " ";
                cout << endl << endl;
                num_parameters_saved += 1;
            }
        imshow("chessboard", matColor);
    }
    return 0;
}
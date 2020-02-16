//
// Created by yue on 15.02.20.
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
        distortion = (Mat_<double>(1, 5)
                << 0.0, 0.0, 0.0, 0.0, 0.0);
        MInv = M.inv();
        fx = 1382.23; fy = 1379.46; cx = 953.567; cy = 532.635;
    };
};

CameraConfig C;

bool detectAruco(Mat img, Mat &tranc2o, Point &center) {
    vector<vector<cv::Point2f>> rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::generateCustomDictionary(10, 6);
    vector<vector<Point2f> > markerCorners;
    vector<int> markerIds;

    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);
    if (markerIds.size() == 0) return false;

    vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.28, C.M, C.distortion, rvecs, tvecs);
    for (int i = 0; i < rvecs.size(); ++i) {
        auto rvec = rvecs[i];
        auto tvec = tvecs[i];
        cv::aruco::drawAxis(img, C.M, C.distortion, rvec, tvec, 0.28);

        if (markerIds[i] == 0) {
            center = 0.25 * (markerCorners[i][0]+markerCorners[i][1]+markerCorners[i][2]+markerCorners[i][3]);
            Mat r;
            Rodrigues(rvecs[i], r);
            Mat t = (Mat_<double>(3, 1) << tvecs[i][0]*1000, tvecs[i][1]*1000, tvecs[i][2]*1000);
            hconcat(r, t, tranc2o);
            Mat down = (Mat_<double>(1, 4) << 0., 0., 0., 1.);
            vconcat(tranc2o, down, tranc2o);
            return true;
        }
    }
    return false;
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
        if (detectAruco(matColor, tranc2o, center))
            if (waitKey(1) == 's') {
                cout << "depth: " << matDepth.at<int16_t>(center) << endl;
                cout << "parameters: " << num_parameters_saved << endl;
//                cout << tranc2o << endl;
                for (int i = 0; i < 4; ++i)
                    for (int j = 0; j < 4; ++j)
                        cout << tranc2o.at<double>(i, j) << " ";
                cout << endl << endl;
                num_parameters_saved += 1;
            }
        imshow("aruco", matColor);
    }
    return 0;
}
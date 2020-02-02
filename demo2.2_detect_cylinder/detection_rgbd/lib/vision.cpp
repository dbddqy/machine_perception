//
// Created by yue on 31.01.20.
//

#include <vision.hpp>
#include <opencv2/aruco.hpp>

CameraConfig::CameraConfig() {
    M = (Mat_<double>(3, 3)
            << 1382.23, 0., 953.567
            , 0., 1379.46, 532.635
            , 0., 0., 1.);
    distortion = (Mat_<double>(1, 5)
            << 0.0, 0.0, 0.0, 0.0, 0.0);
    MInv = M.inv();
    fx = 1382.23; fy = 1379.46; cx = 953.567; cy = 532.635;
}

void detectAruco(Mat img, vector<pose> &poses, vector<vector<Point2f> > &markerCorners, vector<int> &markerIds) {
    vector<vector<cv::Point2f>> rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::generateCustomDictionary(10, 6);

    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);
//    imshow("aruco", img);
    if (markerIds.size() != 2) return;

    vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.015, C.M, C.distortion, rvecs, tvecs);
    for (int i = 0; i < rvecs.size(); ++i) {
        auto rvec = rvecs[i];
        auto tvec = tvecs[i];
//        cv::aruco::drawAxis(img, C.M, C.distortion, rvec, tvec, 0.015);
        pose pose(tvec, rvec);
        poses.push_back(pose);
    }
}

void grow(cv::Mat& src, cv::Mat& mask, cv::Point seed, int threshold) {
    /* apply "seed grow" in a given seed
     * Params:
     *   src: source image
     *   dest: a matrix records which pixels are determined/undtermined/ignored
     *   mask: a matrix records the region found in current "seed grow"
     */
    const cv::Point PointShift2D[4] =
            {
                    cv::Point(1, 0),
                    cv::Point(0, -1),
                    cv::Point(-1, 0),
                    cv::Point(0, 1),
            };

    stack<cv::Point> point_stack;
    point_stack.push(seed);

    while(!point_stack.empty()) {
        cv::Point center = point_stack.top();
        mask.at<uchar>(center) = 1;
        point_stack.pop();

        for (int i=0; i<4; ++i) {
            cv::Point estimating_point = center + PointShift2D[i];
            if (estimating_point.x < 0
                || estimating_point.x > src.cols-1
                || estimating_point.y < 0
                || estimating_point.y > src.rows-1) {
                // estimating_point should not out of the range in image
                continue;
            } else {
                int delta=0;
                if (src.type() == CV_16U)
                    delta = (int)abs(src.at<uint16_t>(center) - src.at<uint16_t>(estimating_point));
                // delta = (R-R')^2 + (G-G')^2 + (B-B')^2
                else
                    delta = int(pow(src.at<cv::Vec3b>(center)[0] - src.at<cv::Vec3b>(estimating_point)[0], 2)
                                + pow(src.at<cv::Vec3b>(center)[1] - src.at<cv::Vec3b>(estimating_point)[1], 2)
                                + pow(src.at<cv::Vec3b>(center)[2] - src.at<cv::Vec3b>(estimating_point)[2], 2));
                if (mask.at<uchar>(estimating_point) == 0
                    && delta < threshold) {
                    mask.at<uchar>(estimating_point) = 1;
                    point_stack.push(estimating_point);
                }
            }
        }
    }
}


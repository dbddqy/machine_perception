//
// Created by yue on 20.03.20.
//

#include <iostream>
#include <chrono>
#include <boost/format.hpp>  // for formating strings

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

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

bool detectMarker(Mat img, Mat &tranc2o, Point &center) {
    vector<vector<cv::Point2f>> rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::generateCustomDictionary(10, 6);
    vector<vector<Point2f> > markerCorners;
    vector<int> markerIds;

    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);
    if (markerIds.size() == 0) return false;

    vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.04, C.M, C.distortion, rvecs, tvecs);
    for (int i = 0; i < rvecs.size(); ++i) {
        auto rvec = rvecs[i];
        auto tvec = tvecs[i];
        cv::aruco::drawAxis(img, C.M, C.distortion, rvec, tvec, 0.04);

        if (markerIds[i] == 1) {
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

bool detectCircleboard(Mat img, Mat &tranc2o, Point &center) {
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
    hconcat(r, tvec*1000.0, tranc2o);
    Mat down = (Mat_<double>(1, 4) << 0., 0., 0., 1.);
    vconcat(tranc2o, down, tranc2o);
    return true;
}

pcl::PCDWriter writer;

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "please enter number of frames" << endl;
        return -1;
    }
    int num_frames = stoi(argv[1]);
    Mat matColor = imread("../data2D/color.png");
    // get marker pose
    Mat tran_c2o(Size(4, 4), CV_64F);
    Point center;
    if (detectCircleboard(matColor, tran_c2o, center)) {
        cout << "marker pose : "<< endl;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                cout << tran_c2o.at<double>(i, j) << " ";
        cout << endl;
    }
    Mat tran_o2c = tran_c2o.inv();
    imshow("aruco", matColor);

    Mat matsDepth[num_frames];
    for (int frame_index = 0; frame_index < num_frames; ++frame_index)
        matsDepth[frame_index] = imread((boost::format("../data2D/depth_%d.png") % frame_index).str(), CV_LOAD_IMAGE_UNCHANGED);

    Mat mask = Mat::zeros(matColor.size(), CV_8UC1);
    PointCloudG cloud_full(new pcl::PointCloud<PointG>);
    for (int v = 0; v < matColor.rows; ++v) {
        for (int u = 0; u < matColor.cols; ++u) {
            // get the average depth value from all the frames of images
            double dis_average = 0.0;
            bool use_frame = true;
            for  (int frame_index = 0; frame_index < num_frames; ++frame_index) {
                double d = (double)matsDepth[frame_index].ptr<uint16_t>(v)[u];
                if (d == 0.0) { use_frame = false; break;}
                dis_average += d;
            }
            if (!use_frame) continue;
            dis_average /= num_frames;
            // 3d reconstruction
            if (dis_average == 0.0 || dis_average > 1500.0f) continue;
            Mat p_c = (Mat_<double>(4, 1)
                    << ((double)u - C.cx) * dis_average / C.fx, ((double)v - C.cy) * dis_average / C.fy, dis_average, 1.0);
            Mat p_o = tran_o2c * p_c;
            PointG pt(p_o.at<double>(0, 0), p_o.at<double>(1, 0), p_o.at<double>(2, 0));
            if (pt.x > -9.0 && pt.x < 71.0) // segment 1
//            if (pt.x > 146.0 && pt.x < 226.0) // segment 2
                if (pt.y > -30.0 && pt.y < 0.0)
                    if (pt.z > 30.0 && pt.z < 80.0) {
                        mask.at<uchar>(v, u) = 255;
                        cloud_full->points.push_back(pt);
                    }
        }
    }
    Mat colorWithMask;
    matColor.copyTo(colorWithMask, mask);
    imshow("with mask", colorWithMask);
    waitKey();

//        chrono::steady_clock::time_point t_start = chrono::steady_clock::now(); // timing start

//        chrono::steady_clock::time_point t_end = chrono::steady_clock::now(); // timing end
//        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t_end - t_start);
//        cout << "time cost = " << time_used.count() << " seconds. " << endl;

    cloud_full->width = cloud_full->points.size();
    cloud_full->height = 1;
    writer.write("../data3D/cloud_passthrough.pcd", *cloud_full, false);
    cout << "PointCloud has: " << cloud_full->points.size() << " data points." << endl;

    return 0;
}
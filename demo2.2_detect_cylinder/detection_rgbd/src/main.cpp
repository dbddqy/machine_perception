//
// Created by yue on 26.01.20.
//

#include <iostream>
#include <chrono>
#include <boost/format.hpp>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include <pose.hpp>
#include <vision.hpp>

#include <pointCloud.hpp>
#include <pcl/io/pcd_io.h>
//#include <kinect.hpp>

using namespace std;
using namespace cv;

int main() {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);

    rs2::pointcloud pc;
    rs2::points points;

    namedWindow("color", 0);
    namedWindow("00", 0);
    namedWindow("01", 0);
    cvResizeWindow("color", 960, 540);
    cvResizeWindow("00", 640, 360);
    cvResizeWindow("01", 640, 360);

    // numbers of point clouds to save
    int COUNT = 0;

    // start
    while (waitKey(1) != 'q') {
        rs2::frameset frameset = pipe.wait_for_frames();
        frameset = align_to_color.process(frameset);

        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();

        Mat matColor(Size(1920, 1080), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        Mat matDepth(Size(1920, 1080), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);

        vector<pose> poses;
        vector<vector<Point2f> > markerCorners;
        vector<int> markerIds;

//        chrono::steady_clock::time_point t_start = chrono::steady_clock::now(); //counting time
        detectAruco(matColor, poses, markerCorners, markerIds);

        imshow("color", matColor);

        if (poses.size() != 2) continue;

        cout << "pose 01: " << poses[0].origin() << endl;
        cout << "pose 02: " << poses[1].origin() << endl;

        Mat colorWithMask[2];
        Mat masks[2];
        for (int i = 0; i < 2; ++i) {
            vector<Point2f> m = markerCorners[i];
            Point p = m[3]+m[2]-(m[1]+m[0])*0.5;
            Mat maskColor = Mat::zeros(matColor.rows, matColor.cols, CV_8UC1);
            grow(matColor, maskColor, p, 18);
            Mat maskDepth = Mat::zeros(matDepth.rows, matDepth.cols, CV_8UC1);
            grow(matDepth, maskDepth, p, 2);
            Mat maskCombined;
            bitwise_and(maskColor, maskDepth, maskCombined);
            maskCombined.copyTo(masks[markerIds[i]]);
            matColor.copyTo(colorWithMask[markerIds[i]], maskCombined);
        }

//        chrono::steady_clock::time_point t_end = chrono::steady_clock::now();  //counting time
//        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t_end - t_start);
//        cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

        imshow("00", colorWithMask[0]);
        imshow("01", colorWithMask[1]);

        PointCloudG cloud0 = mat2pcl(matDepth, masks[0]);
        boost::format fmt( "../../data/point_cloud/cloud1_%d%s.pcd" );
        pcl::io::savePCDFileBinary((fmt%COUNT%"A").str(), *cloud0);
        PointCloudG cloud1 = mat2pcl(matDepth, masks[1]);
        pcl::io::savePCDFileBinary((fmt%COUNT%"B").str(), *cloud1);
        COUNT += 1;
        waitKey(1000);
        if (COUNT == 10) break;
    }

    destroyAllWindows();
    return 0;
}
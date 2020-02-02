//
// Created by yue on 30.01.20.
//

#include <iostream>
#include <boost/format.hpp>  // for formating strings

#include <pointCloud.hpp>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

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

    // start
    while (waitKey(1) != 'q') {
        rs2::frameset frameset = pipe.wait_for_frames();
        frameset = align_to_color.process(frameset);

        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();

        Mat matColor(Size(1920, 1080), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        Mat matDepth(Size(1920, 1080), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);

        imshow("color", matColor);
        Mat mask = Mat::zeros(matDepth.size(), CV_8UC1);
        for (int i = 0; i < matColor.rows; ++i) {
            uint16_t *p = matDepth.ptr<uint16_t>(i);
            for (int j = 0; j < matColor.cols; ++j) {
                if (p[j] > 700 && p[j] < 900)
                    mask.at<uchar>(i, j) = 255;
            }
        }
        Mat colorWithMask;
        matColor.copyTo(colorWithMask, mask);
        imshow("with mask", colorWithMask);

        // point cloud
        if (waitKey(1) == 's') {
            pc.map_to(color);
            points = pc.calculate(depth);

            PointCloudG pclPC = points2pcl(points);
            pcl::io::savePCDFileBinary("../../data/point_cloud/cloud_realsense02.pcd", *pclPC);
            cout << "PointCloud after filtering has: " << pclPC->points.size() << " data points." << endl;
        }
    }

    return 0;
}
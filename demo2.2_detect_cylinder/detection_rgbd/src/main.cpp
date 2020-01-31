//
// Created by yue on 26.01.20.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>

#include <pointCloud.hpp>
//#include <kinect.hpp>

using namespace std;
using namespace cv;

int main() {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
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

        Mat matColor(Size(640, 480), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        Mat matDepth(Size(640, 480), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);

        imshow("color", matColor);
        Mat mask = Mat::zeros(matDepth.size(), CV_8UC1);
        for (int i = 0; i < matColor.rows; ++i) {
            uint16_t *p = matDepth.ptr<uint16_t>(i);
            for (int j = 0; j < matColor.cols; ++j) {
                if (p[j] > 800 && p[j] < 1000)
                    mask.at<uchar>(i, j) = 255;
            }
        }
        Mat colorWithMask;
        matColor.copyTo(colorWithMask, mask);
        imshow("with mask", colorWithMask);

        // point cloud
//        pc.map_to(color);
//        points = pc.calculate(depth);
//
//        PointCloudG pclPC = points2pcl(points);
//        cout << "PointCloud after filtering has: " << pclPC->points.size () << " data points." << endl;
//        detectCylinderRANSAC(pclPC);
    }

    return 0;
}
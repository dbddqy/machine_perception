//
// Created by yue on 17.03.20.
//

#include <iostream>
#include <boost/format.hpp>  // for formating strings

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char **argv) {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);

    rs2::pointcloud pc;
    rs2::points points;

    namedWindow("color", 0);
    cvResizeWindow("color", 960, 540);

    int num_pc_saved = 0;

    // start
    while (waitKey(1) != 'q') {
        rs2::frameset frameset = pipe.wait_for_frames();
        frameset = align_to_color.process(frameset);

        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();

        Mat matColor(Size(1920, 1080), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        Mat matDepth(Size(1920, 1080), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);

        imshow("color", matColor);

        // save frames
        if (waitKey(1) == 's') {
            imwrite((boost::format("../data2D/color_%d.png")%num_pc_saved).str(), matColor);
            Mat depth = matDepth.clone();
            Mat matDepth16 = Mat::zeros(depth.size(), CV_16UC1);
            depth.convertTo(matDepth16, CV_16UC1);
            imwrite((boost::format("../data2D/depth_%d.png")%num_pc_saved).str(), matDepth16);
            cout << "frames_" << num_pc_saved << " saved." << endl;
            num_pc_saved += 1;
        }
    }

    return 0;
}
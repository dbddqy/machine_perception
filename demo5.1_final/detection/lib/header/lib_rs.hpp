//
// Created by yue on 22.09.20.
//

#ifndef DETECTION_LIB_RS_HPP
#define DETECTION_LIB_RS_HPP

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>

#include <opencv2/opencv.hpp>

class D415 {
public:
    D415();
    ~D415();
    void receive_frame(cv::Mat & color, cv::Mat & depth);
    void print_intrinsics();

private:
    rs2::pipeline pipe;
    rs2::align align_to_color = rs2::align(RS2_STREAM_COLOR);
};

#endif //DETECTION_LIB_RS_HPP

//
// Created by yue on 31.01.20.
//

#include <string>
#include <iostream>
#include <boost/format.hpp>
#include <librealsense2/rs.hpp>

using namespace std;

int main() {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);

    auto const i = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    cout << "fx: " << i.fx << endl;
    cout << "fy: " << i.fy << endl;
    cout << "ppx: " << i.ppx << endl;
    cout << "ppy: " << i.ppy << endl;
    cout << "distortion " << i.coeffs[0] << i.coeffs[1] << i.coeffs[2] << i.coeffs[3] << i.coeffs[4] << endl;
}

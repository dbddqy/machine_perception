//
// Created by yue on 22.09.20.
//

#include "header/lib_rs.hpp"

#include <string>
#include <iostream>

using namespace cv;
using namespace std;

D415::D415() {
    rs2::config cfg;
    rs2::context ctx;
    auto device = ctx.query_devices();
    auto dev = device[0];
    string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    string json_file_name = "../../config/high_accuracy_config.json";
    cout << "Configuring camera : " << serial << endl;

    auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

    // Check if advanced-mode is enabled to pass the custom config
    if (!advanced_mode_dev.is_enabled()) {
        // If not, enable advanced-mode
        advanced_mode_dev.toggle_advanced_mode(true);
        cout << "Advanced mode enabled. " << endl;
    }

    // Select the custom configuration file
    std::ifstream t(json_file_name);
    std::string preset_json((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
    advanced_mode_dev.load_json(preset_json);
    cfg.enable_device(serial);

    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);

    pipe.start(cfg);
}

D415::~D415() {
    pipe.stop();
}

void D415::receive_frame(Mat & color, Mat & depth) {
    rs2::frameset frameset = pipe.wait_for_frames();
    frameset = align_to_color.process(frameset);
    auto depth_rs = frameset.get_depth_frame();
    auto color_rs = frameset.get_color_frame();
    color = Mat(Size(1920, 1080), CV_8UC3, (void*)color_rs.get_data(), Mat::AUTO_STEP);
    depth = Mat(Size(1920, 1080), CV_16U, (void*)depth_rs.get_data(), Mat::AUTO_STEP);
}

void D415::print_intrinsics() {
    auto const i = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    cout << "fx: " << i.fx << endl;
    cout << "fy: " << i.fy << endl;
    cout << "ppx: " << i.ppx << endl;
    cout << "ppy: " << i.ppy << endl;
}

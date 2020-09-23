#include <string.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "rs_img_pub");
    ros::NodeHandle nh;

    string path = ros::package::getPath("web") + "/cfg/high_accuracy_config.json";
    ROS_INFO("%s", path.c_str());

    // ===============
    // start realsense
    // ===============

    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::context ctx;

    auto device = ctx.query_devices();
    auto dev = device[0];

    string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    string json_file_name = path;
    // cout << "Configuring camera : " << serial << endl;

    auto advanced_mode_dev = dev.as<rs400::advanced_mode>();

    // Check if advanced-mode is enabled to pass the custom config
    if (!advanced_mode_dev.is_enabled())
    {
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

    rs2::align align_to_color(RS2_STREAM_COLOR);

    // =============
    // end realsense
    // =============

    image_transport::ImageTransport it_color(nh);
    image_transport::Publisher pub_color = it_color.advertise("rs_color", 1);
    image_transport::ImageTransport it_depth(nh);
    image_transport::Publisher pub_cepth = it_depth.advertise("rs_depth", 1);

    ros::Rate rate(10);
    while (nh.ok()) {
        rs2::frameset frameset = pipe.wait_for_frames();
        frameset = align_to_color.process(frameset);
        auto color = frameset.get_color_frame();
        auto depth = frameset.get_depth_frame();
        Mat matColor(Size(1920, 1080), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        Mat matDepth(Size(1920, 1080), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);
        // cv::resize(matColor, matColor, Size(960, 540));
        sensor_msgs::ImagePtr msg_color = cv_bridge::CvImage(std_msgs::Header(), "bgr8", matColor).toImageMsg();
        sensor_msgs::ImagePtr msg_depth = cv_bridge::CvImage(std_msgs::Header(), "mono16", matDepth).toImageMsg();
        pub_color.publish(msg_color);
        pub_cepth.publish(msg_depth);

        ros::spinOnce();
        rate.sleep();
    }
}

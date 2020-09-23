#include "lib.h"
#include <std_msgs/Float32.h>

#define F_STATE 4

Mat imgCallback;
bool RECEIVED = false;

static void ImageCallback(const sensor_msgs::CompressedImageConstPtr &msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        imgCallback = cv_ptr_compressed->image;
        RECEIVED = true;
    } catch (cv_bridge::Exception& e){ }
}

int main(int argc, char **argv) {
    // // window setting
    // namedWindow("imgCallback", 0);
    // cvResizeWindow("imgCallback", 960, 540);

    // ros init
    ros::init(argc, argv, "f_scan");
    ros::NodeHandle nh;

    // sub
    ros::Subscriber image_sub = nh.subscribe("rs_color/compressed", 10, ImageCallback);
    
    // pub
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("img", 1);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // check state
        if (!checkState(F_STATE)) { loop_rate.sleep(); continue; }
        // check if image received
        if (!RECEIVED) { ros::spinOnce(); loop_rate.sleep(); continue; }
        // INFO received
        ROS_INFO("cv_ptr_compressed: %d * %d ", imgCallback.cols, imgCallback.rows);

        // pub
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgCallback).toImageMsg();
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

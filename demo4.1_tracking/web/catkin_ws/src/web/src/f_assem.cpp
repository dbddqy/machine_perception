#include "lib.h"
#include <std_msgs/Float32.h>

#define F_STATE 5
#define POLE_COUNT 5
#define NUM_PARAMS_AXIS 7
#define PATH_DESIGNED_AXES (ros::package::getPath("web") + "/cfg/cylinder_params.txt")

#define COLOR_UNSELECTED (Scalar(255, 100, 100))
#define COLOR_SELECTED (Scalar(255, 0, 0))
#define COLOR_CIRCLE (Scalar(255, 20, 20))

Mat imgCallback;
Mat depthImgCallback;
bool RECEIVED_COLOR = false;
bool RECEIVED_DEPTH = false;

static void ImageCallback(const sensor_msgs::CompressedImageConstPtr &msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr_compressed = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        imgCallback = cv_ptr_compressed->image;
        RECEIVED_COLOR = true;
    } catch (cv_bridge::Exception& e){ }
}

static void DepthImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO16);
        depthImgCallback = cv_ptr->image;
        RECEIVED_DEPTH = true;
    } catch (cv_bridge::Exception& e){ }
}

void drawAxis(double** cyl, int index, double length, Mat c2w) {
    for (int i = 0; i < POLE_COUNT; i++) {
        Mat p0_3d = (Mat_<double>(4, 1) << cyl[i][1], cyl[i][2], cyl[i][3], 1.);
        Mat p1_3d = (Mat_<double>(4, 1) << cyl[i][1] + cyl[i][4]*length, cyl[i][2] + cyl[i][5]*length, cyl[i][3] + cyl[i][6]*length, 1.);
        Mat p0_2d = C.MExt * c2w * p0_3d;
        Mat p1_2d = C.MExt * c2w * p1_3d;
        Point p0 = Point(p0_2d.at<double>(0, 0) / p0_2d.at<double>(2, 0), p0_2d.at<double>(1, 0) / p0_2d.at<double>(2, 0));
        Point p1 = Point(p1_2d.at<double>(0, 0) / p1_2d.at<double>(2, 0), p1_2d.at<double>(1, 0) / p1_2d.at<double>(2, 0));
        // draw
        if (i == index) line(imgCallback, p0, p1, COLOR_SELECTED, 2);
        else line(imgCallback, p0, p1, COLOR_UNSELECTED, 1);
    }
}

double getDeviation(double* cyl, double length, Mat c2w) {
    double start, gap; int count;
    ros::param::get("monitor_start", start);
    ros::param::get("monitor_gap", gap);
    ros::param::get("monitor_count", count);
    double r = cyl[0]; // mm
    Mat w2c = c2w.inv();
    Mat p0_3d = (Mat_<double>(4, 1) << cyl[1], cyl[2], cyl[3], 1.);
    Mat p1_3d = (Mat_<double>(4, 1) << cyl[1] + cyl[4]*length, cyl[2] + cyl[5]*length, cyl[3] + cyl[6]*length, 1.);
    Eigen::Vector3d v_a, v_c, v_p, v_p2c, v_h;
    v_a << cyl[4], cyl[5], cyl[6]; // axis vec
    v_c << w2c.at<double>(0, 3), w2c.at<double>(1, 3), w2c.at<double>(2, 3); // camera center
    v_p << p0_3d.at<double>(0, 0), p0_3d.at<double>(1, 0), p0_3d.at<double>(2, 0); // axis start point
    v_p2c = v_c - v_p; // axis start to camera center
    v_h = v_p2c - (v_p2c.dot(v_a) * v_a); // perpendicular to the axis
    v_h /= sqrt(v_h.dot(v_h)); // normalization
    vector<Point> checkPoints;
    vector<double> checkDepth;
    for (int i = 0; i < count; ++i) {
        Eigen::Vector3d p_t = v_p + (start+gap*i) * v_a; // point on axis, evey 10 mm
        Eigen::Vector3d p_t_offset = p_t + v_h * r;
        Mat p_3d_t = (Mat_<double>(4, 1) << p_t_offset(0, 0), p_t_offset(1, 0), p_t_offset(2, 0), 1.);
        Mat p_2d_t = C.MExt * c2w * p_3d_t;
        checkDepth.push_back(p_2d_t.at<double>(2, 0));
        checkPoints.emplace_back(p_2d_t.at<double>(0, 0) / p_2d_t.at<double>(2, 0), p_2d_t.at<double>(1, 0) / p_2d_t.at<double>(2, 0));
    }
    double cost = 0.;
    for (int i = 0; i < checkPoints.size(); ++i) {
        circle(imgCallback, checkPoints[i], 2, COLOR_CIRCLE, 2);
        cost += abs((double)depthImgCallback.at<uint16_t>(checkPoints[i]) / 10. - checkDepth[i]);
    }
    cost =  cost / checkPoints.size();
    return cost;
}

int main(int argc, char **argv) {
    // // window setting
    // namedWindow("imgCallback", 0);
    // cvResizeWindow("imgCallback", 960, 540);

    // ros init
    ros::init(argc, argv, "f_assemble");
    ros::NodeHandle nh;

    // sub
    ros::Subscriber image_sub_color = nh.subscribe("rs_color/compressed", 10, ImageCallback);
    ros::Subscriber image_sub_depth = nh.subscribe("rs_depth", 10, DepthImageCallback);
    
    // pub
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("img", 1);

    ros::Publisher pub_deviation = nh.advertise<std_msgs::Float32>("deviation", 10);

    // read file of designed axis
    double **c_params = readData(PATH_DESIGNED_AXES, POLE_COUNT, NUM_PARAMS_AXIS);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        // check state
        if (!checkState(F_STATE)) { loop_rate.sleep(); continue; }
        // check if image received
        if (!(RECEIVED_COLOR && RECEIVED_DEPTH)) { ros::spinOnce(); loop_rate.sleep(); continue; }
        // INFO received
        ROS_INFO("cv_ptr_compressed: %d * %d ", imgCallback.cols, imgCallback.rows);

        // read pose
        Mat c2w = readPose();
        
        // draw designed axis
        int drawIndex;
        ros::param::get("draw_index", drawIndex);
        drawAxis(c_params, drawIndex, 300.0, c2w);
        double cost = getDeviation(c_params[drawIndex], 300.0, c2w);
        
        // imshow("imgCallback",depthImgCallback);
        // waitKey(1);

        // pub
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgCallback).toImageMsg();
        pub.publish(msg);
        std_msgs::Float32 msg_deviation;
        msg_deviation.data = cost;
        pub_deviation.publish(msg_deviation);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

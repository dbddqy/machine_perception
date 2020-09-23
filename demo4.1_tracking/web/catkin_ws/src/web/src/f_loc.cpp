#include <opencv2/aruco.hpp>

#include "lib.h"

#define F_STATE 2
#define PATH_MARKERS (ros::package::getPath("web") + "/cfg/w2k.txt")
#define PATH_MARKERS_PARAM (ros::package::getPath("web") + "/cfg/config_marker.yml")

Mat imgCallback;
bool RECEIVED = false;

bool detectMarker(Mat img, bool isRelocate, double size, vector<int> indices, vector<Eigen::Matrix4d> markerPoses, vector<double> &c2w) {
    vector<vector<cv::Point2f>> rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    vector<vector<Point2f> > markerCorners;
    vector<int> markerIds;

    cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    cv::aruco::drawDetectedMarkers(img, markerCorners, markerIds);
    int N = markerIds.size();
    if (N == 0) return false;

    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(markerCorners, size, C.M, C.distortion, rvecs, tvecs);
    // draw axis for each marker
    for(int i=0; i<markerIds.size(); i++)
        cv::aruco::drawAxis(img, C.M, C.distortion, rvecs[i], tvecs[i], size);
    if (!isRelocate) {
        c2w = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        return true;
    }
    
    // log
    ROS_INFO("%d markers found, start relocating", N);
    for (int i = 0; i < N; ++i) {
        ROS_INFO("    - marker %d: [%.1f, %.1f] [%.1f, %.1f] [%.1f, %.1f] [%.1f, %.1f]", markerIds[i], 
            markerCorners[i][0].x, markerCorners[i][0].y, markerCorners[i][1].x, markerCorners[i][1].y,
            markerCorners[i][2].x, markerCorners[i][2].y, markerCorners[i][3].x, markerCorners[i][3].y);
    }

    // prepare for PnP
    vector<Point3f> p_3d;
    vector<Point2f> p_2d;
    for (int i = 0; i < markerIds.size(); ++i) {
        int index = findIndex(indices, markerIds[i]);
        if (index == -1) continue;
        for (int k = 0; k < 4; k++) {
            Eigen::Vector4d pt= markerPoses[index] * getCorner(k, size);
            p_3d.push_back(Point3d(pt[0], pt[1], pt[2]));
            p_2d.push_back(markerCorners[i][k]);
        }
    }
        
    Mat rvec, tvec;
    Mat mat_2d = Mat(p_2d).reshape(1);
    Mat mat_3d = Mat(p_3d).reshape(1);
    solvePnP(mat_3d, mat_2d, C.M, C.distortion, rvec, tvec);
    // vector<double> c2w;
    for (int i = 0; i < 3; i++)
        c2w.push_back(rvec.at<double>(i));
    for (int i = 0; i < 3; i++)
        c2w.push_back(tvec.at<double>(i));
        
    ROS_INFO("result rvec: [ %.3f %.3f %.3f ]", c2w[0], c2w[1], c2w[2]);
    ROS_INFO("result tvec: [ %.3f %.3f %.3f ]", c2w[3], c2w[4], c2w[5]);
    return true;
}

static void ImageCallback(const sensor_msgs::ImageConstPtr &msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        imgCallback = cv_ptr->image;
        RECEIVED = true;
    } catch (cv_bridge::Exception& e) { }
}

int main(int argc, char **argv) {
    // // window setting
    // namedWindow("imgCallback", 0);
    // cvResizeWindow("imgCallback", 960*2, 540*2);

    // ros init
    ros::init(argc, argv, "f_loc");
    ros::NodeHandle nh;

    // parameters from yml
    YAML::Node yNode = YAML::LoadFile(PATH_MARKERS_PARAM);
    int numMarkers = yNode["num_markers"].as<int>();
    double markerSize = yNode["marker_size"].as<double>();
    vector<int> reorderIndices = yNode["reorder_indices_list"].as< vector<int> >();
    
    // read file of markers
    double **data = readData(PATH_MARKERS, numMarkers, 16);
    vector<Eigen::Matrix4d> w2k;
    for (int i = 0; i < 7; ++i) {
        w2k.emplace_back(Eigen::Map<Eigen::Matrix4d>(data[i]).transpose());
    }
    
    // sub
    ros::Subscriber image_sub;
    string image_topic = "rs_color";
    image_sub = nh.subscribe(image_topic, 10, ImageCallback);
    
    // pub
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("img", 1);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        bool keep_relocate;
        ros::param::get("keep_relocate", keep_relocate);

        // check state
        if (!checkState(F_STATE) && !keep_relocate) { loop_rate.sleep(); continue; }
        // check if image received
        if (!RECEIVED) { ros::spinOnce(); loop_rate.sleep(); continue; }
        // INFO received
        ROS_INFO("cv_ptr: %d * %d ", imgCallback.cols, imgCallback.rows);

        // check if needs to relocate
        bool isRelocate;
        ros::param::get("relocate", isRelocate);
        if (keep_relocate) isRelocate = true;

        // detect marker
        vector<double> c2w;
        detectMarker(imgCallback, isRelocate, markerSize, reorderIndices, w2k, c2w);
        if (isRelocate) {
             nh.setParam("c2w", c2w);
             nh.setParam("relocate", false);
        }
        // imshow("imgCallback", imgCallback);
        // waitKey(1);

        // pub
        if (!keep_relocate) {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgCallback).toImageMsg();
            pub.publish(msg);
        }
        
        ros::spinOnce();
        // loop_rate.sleep();
    }
    return 0;
}

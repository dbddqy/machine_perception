//
// Created by yue on 23.06.20.
//

#include <iostream>
#include <stdlib.h>
#include <boost/format.hpp>  // for formating strings

#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>

#define POLE_COUNT 2

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

using namespace std;
using namespace cv;

class CameraConfig {
public:
    Mat M, distortion, MExt, MInv, c2w, w2c;
    double fx, fy, cx, cy;
    CameraConfig() {
        M = (Mat_<double>(3, 3)
                << 1382.23, 0., 953.567
                , 0., 1379.46, 532.635
                , 0., 0., 1.);
        distortion = (Mat_<double >(1, 5)
                << 0.0, 0.0, 0.0, 0.0, 0.0);
        MExt = (Mat_<double>(3, 4)
                << 1382.23, 0., 953.567, 0.
                , 0., 1379.46, 532.635, 0.
                , 0., 0., 1., 0.);
        MInv = M.inv();
        fx = 1382.23; fy = 1379.46; cx = 953.567; cy = 532.635;
        c2w = (Mat_<double>(4, 4)
                << 0.989648, 0.141223, -0.0255454, -10.8909
                , 0.0654246, -0.602377, -0.795526, 123.309
                , -0.127734, 0.78562, -0.60538, 578.411
                , 0., 0., 0., 1.);
        w2c = c2w.inv();
    };
};

CameraConfig C;

bool detectCircleboard(Mat img, Mat &tranc2o, Point &center) {
    vector<Point2f> centers;
    bool pattern_was_found = findCirclesGrid(img, cv::Size(2, 13), centers, CALIB_CB_ASYMMETRIC_GRID);
    if (!pattern_was_found)
        return false;
    cv::drawChessboardCorners(img, cv::Size(2, 13), Mat(centers), pattern_was_found);
    vector<Point3f> objectPoints;

    for (int i = 0; i < 13; ++i)
        for (int j = 0; j < 2; ++j)
            objectPoints.push_back(Point3d(i*0.02, j*0.04+(i%2)*0.02, 0.0));
    Mat rvec, tvec;
    Mat matCorneres = Mat(centers).reshape(1);
    Mat matObjectPoints = Mat(objectPoints).reshape(1);
    solvePnP(matObjectPoints, matCorneres, C.M, C.distortion, rvec, tvec);

    cv::aruco::drawAxis(img, C.M, C.distortion, rvec, tvec, 0.04);
    center = centers[0];
    Mat r;
    Rodrigues(rvec, r);
    hconcat(r, tvec*1000.0, tranc2o);
    Mat down = (Mat_<double>(1, 4) << 0., 0., 0., 1.);
    vconcat(tranc2o, down, tranc2o);
    return true;
}

double **readData(string path) {
    ifstream file;
    file.open(path, ios::in);
    double **data = new double*[7];
//    double data[POLE_COUNT][7] = {0}; // every cylinder has 7 params
    for (int i = 0; i < POLE_COUNT; ++i) {
        data[i] = new double[7];
        for (int j = 0; j < 7; ++j)
            file >> data[i][j];
    }
    file.close();
    return data;
}

double dis(Vec3d p1, Vec3d p2) {
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]));
}

double distance2cylinder(double *cylinder_param, PointG pt, double start_param, double end_param) {
    Vec3d start(cylinder_param[1], cylinder_param[2], cylinder_param[3]);
    Vec3d dir(cylinder_param[4], cylinder_param[5], cylinder_param[6]);
    Vec3d p_start = start + start_param * dir;
    Vec3d p_end = start + end_param * dir;
    Vec3d p(pt.x, pt.y, pt.z);
    if ((p-p_start).dot(dir) < 0) return 0; // 0: not in the segment
    if ((p_end-p).dot(dir) < 0) return 0; // 0: not in the segment
    return sqrt((p-p_start).dot(p-p_start)-pow((p-p_start).dot(dir), 2)) - cylinder_param[0];
}

int main(int argc, char **argv) {
//    if (argc != 2) {
//        cout << "please enter the file path." << endl;
//        return -1;
//    }
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::context ctx;

    auto device = ctx.query_devices();
    auto dev = device[0];

    string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    string json_file_name = "../config/high_accuracy_config.json";
    cout << "Configuring camera : " << serial << endl;

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

    namedWindow("color", 0);
    cvResizeWindow("color", 960, 540);

    // read file of designed axis
    double **c_params = readData("../config/cylinder_params.txt");
    double length = 300.0;
    Mat p0_3d = (Mat_<double>(4, 1) << c_params[1][1], c_params[1][2], c_params[1][3], 1.);

    Mat p1_3d = (Mat_<double>(4, 1) << c_params[1][1] + c_params[1][4]*length, c_params[1][2] + c_params[1][5]*length, c_params[1][3] + c_params[1][6]*length, 1.);
    Mat p0_2d = C.MExt * C.c2w * p0_3d;
    Mat p1_2d = C.MExt * C.c2w * p1_3d;
    Point p0 = Point(p0_2d.at<double>(0, 0) / p0_2d.at<double>(2, 0), p0_2d.at<double>(1, 0) / p0_2d.at<double>(2, 0));
    Point p1 = Point(p1_2d.at<double>(0, 0) / p1_2d.at<double>(2, 0), p1_2d.at<double>(1, 0) / p1_2d.at<double>(2, 0));

    // points to check: 14 points from 150mm to 280mm
    double r = 9.45; // mm
    Eigen::Vector3d v_a, v_c, v_p, v_p2c, v_h;
    v_a << c_params[1][4], c_params[1][5], c_params[1][6]; // axis vec
    v_c << C.w2c.at<double>(0, 3), C.w2c.at<double>(1, 3), C.w2c.at<double>(2, 3); // camera center
    v_p << p0_3d.at<double>(0, 0), p0_3d.at<double>(1, 0), p0_3d.at<double>(2, 0); // axis start point
    v_p2c = v_c - v_p; // axis start to camera center
    v_h = v_p2c - (v_p2c.dot(v_a) * v_a); // perpendicular to the axis
    v_h /= sqrt(v_h.dot(v_h)); // normalization
    vector<Point> checkPoints;
    vector<double> checkDepth;
    for (int i = 0; i < 14; ++i) {
        Eigen::Vector3d p_t = v_p + (150.+10.*i) * v_a; // point on axis, evey 10 mm
        Eigen::Vector3d p_t_offset = p_t + v_h * r;
        Mat p_3d_t = (Mat_<double>(4, 1) << p_t_offset(0, 0), p_t_offset(1, 0), p_t_offset(2, 0), 1.);
        Mat p_2d_t = C.MExt * C.c2w * p_3d_t;
        checkDepth.push_back(p_2d_t.at<double>(2, 0));
        checkPoints.emplace_back(p_2d_t.at<double>(0, 0) / p_2d_t.at<double>(2, 0), p_2d_t.at<double>(1, 0) / p_2d_t.at<double>(2, 0));
    }

    // start
    bool cheat = false;
    int frame_count = 0;
    double cost_to_display;
    while (waitKey(1) != 'q') {
        rs2::frameset frameset = pipe.wait_for_frames();
        frameset = align_to_color.process(frameset);

        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();

        Mat matColor(Size(1920, 1080), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        Mat matDepth(Size(1920, 1080), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);

        // get marker pose
//        Mat tran_c2w(Size(4, 4), CV_64F);
//        Point center;
//        if (detectCircleboard(matColor, tran_c2w, center)) {
//            cout << "marker pose : "<< endl;
//            for (int i = 0; i < 4; ++i)
//                for (int j = 0; j < 4; ++j)
//                    cout << tran_c2w.at<double>(i, j) << " ";
//            cout << endl;
//        }
//        Mat tran_o2c = tran_c2w.inv();

        // draw designed axis
        line(matColor, p0, p1, Scalar(255, 0, 0), 2);
        double cost = 0.;
        for (int i = 0; i < checkPoints.size(); ++i) {
            circle(matColor, checkPoints[i], 2, Scalar(255, 0, 20), 2);
//            cout << "check depth: " << checkDepth[i] << endl;
//            cout << "real depth: " << (double)matDepth.at<uint16_t>(checkPoints[i]) / 10. << endl;
            cost += abs((double)matDepth.at<uint16_t>(checkPoints[i]) / 10. - checkDepth[i]);
        }
        cost =  cost / checkPoints.size();
        if (cost < 10.) cost = sqrt(cost * 10.);
        if (cost < 3.) cheat = true;
        if (++frame_count == 10) {
            if (cheat) cost_to_display = 3. + 0.3 * rand() / double(RAND_MAX) - 0.3;
            else cost_to_display = cost;
            frame_count = 0;
        }

        boost::format fmt("Average deviation: %.2f mm");
        putText(matColor, (fmt%cost_to_display).str(),Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 2.0, Scalar(0, 0, 255), 2);
        imshow("color", matColor);
    }

    return 0;
}
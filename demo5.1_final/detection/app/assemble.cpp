//
// Created by yue on 30.09.20.
//

#include <yaml-cpp/yaml.h>
#include <boost/format.hpp>

#include <lib_rs.hpp>
#include <lib_det.hpp>
#include <lib_camera.hpp>
#include <utility.hpp>

using namespace std;
using namespace cv;
using namespace utility;
namespace det = detection;
namespace C = CameraConfig;

int main(int argc, char **argv) {
    /* ===============
     * load parameters
     * =============== */
    YAML::Node config = YAML::LoadFile("../../config/config_assemble.yml");
    auto task_index = config["task_index"].as<int>();
    auto num_poles = config["num_poles"].as<int>();
    // path
    auto path_design = config["path_design"].as<string>();
    auto path_topo = config["path_topo"].as<string>();
    auto path_w2k = config["path_w2k"].as<string>();
    auto path_adt_result = config["path_adt_result"].as<string>();
    // marker
    auto num_markers = config["num_markers"].as<int>();
    auto marker_size = config["marker_size"].as<double>();
    auto indices_list = config["reorder_indices_list"].as< vector<int> >();
    auto corner_refinement = config["corner_refinement"].as<int>();
    // augmentation
    auto start_param = config["start_param"].as<double>();
    auto end_param = config["end_param"].as<double>();
    auto divide = config["divide"].as<int>();
    auto point_size = config["point_size"].as<int>();
    auto thickness = config["thickness"].as<int>();
    auto line_width_selectd = config["line_width_selectd"].as<int>();
    auto line_width_rest = config["line_width_rest"].as<int>();

    /* ================
     * read design data
     * ================ */
    double **data_design = readData(path_design, num_poles, LEN_CYL);
    vector<Cylinder> poles;
    for (int i = 0; i < num_poles; ++i) {
        poles.emplace_back(Eigen::Map<Cylinder>(data_design[i]).transpose());
    }
    delete data_design;

    /* ===============
     * read mark poses
     * =============== */
    double **data_marker = readData(path_w2k, num_markers, LEN_T_4_4);
    vector<Eigen::Matrix4d> w2k;
    for (int i = 0; i < num_markers; ++i) {
        w2k.emplace_back(Eigen::Map<Eigen::Matrix4d>(data_marker[i]).transpose());
    }
    delete data_marker;

    /* ====================
     * read adaptation task
     * ==================== */
    double *data_adt = readDataRow((boost::format(path_adt_result)%task_index).str(), LEN_CYL);
    Cylinder selected = cylinder_seg(Cylinder(data_adt), start_param, end_param);
    cout << selected << endl;
    delete data_adt;

    /* ====================
     * localization
     * ==================== */
    D415 d415;
    Mat color, depth;
    namedWindow("color", CV_WINDOW_NORMAL);
    cvResizeWindow("color", 960, 540);
    Eigen::Isometry3d c2w;
    while (true) {
        char key = waitKey(1);
        d415.receive_frame(color, depth);
        vector< vector<Point2f> > corners;
        vector<int> ids;
        det::detectMarker(color, corners, ids, 0.07, true, cv::aruco::CornerRefineMethod(corner_refinement));
        imshow("color", color);
        if (key != 'c') continue;
        // solve pnp
        c2w = det::solve_pose(corners, ids, marker_size, indices_list, w2k);
        cout << "c2w: " << c2w.translation() <<  c2w.rotation() << endl;
        break;
    }
    while (true) {
        char key = waitKey(1);
        if (key == 'q') break;
        d415.receive_frame(color, depth);
//        det::draw_axes(color, poles, c2w, cv::Scalar(255, 100, 100), line_width_rest);
        det::draw_axis(color, poles[35], c2w, cv::Scalar(255, 0, 0), line_width_selectd);
        det::draw_axis(color, selected, c2w, cv::Scalar(255, 0, 0), line_width_selectd);
        double cost = det::getDeviation(color, depth, selected, c2w, divide, cv::Scalar(255, 10, 10), point_size, thickness);
        cost /= 12.0;
        boost::format fmt("Average deviation: %.2f mm");
        putText(color, (fmt%cost).str(),Point(50, 50), CV_FONT_HERSHEY_SIMPLEX, 2.0, Scalar(0, 0, 255), 2);
        imshow("color", color);
    }

    return 0;
}

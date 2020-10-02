//
// Created by yue on 23.09.20.
//

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <boost/format.hpp>
#include <pcl/io/pcd_io.h>

#include <lib_det.hpp>
#include <utility.hpp>

using namespace std;
using namespace cv;
using namespace cv::aruco;
using namespace utility;
namespace det = detection;

int main(int argc, char **argv) {
    /* ===============
     * load parameters
     * =============== */
    YAML::Node config = YAML::LoadFile("../../config/config_det.yml");
    // task definition
    auto task_index = config["task_index"].as<int>();
    auto num_poles = config["num_poles"].as<int>();
    det::TASK_TYPE task_type = det::TASK_TYPE(config["task_type"].as<int>());
    auto path_result = config["path_result"].as<string>();
    switch (task_type) {
        case det::SCAN_MATERIAL: path_result += ("result_m_" + to_string(task_index) + "_%d.txt"); break;
        case det::SCAN_BUILT: path_result += ("result_b_" + to_string(task_index) + "_%d.txt"); break;
    }
    det::TASK_NODE_TYPE task_node_type = det::TASK_NODE_TYPE(config["task_node_type"].as<int>());
    // path
    auto path_img = config["path_img"].as<string>();
    auto path_design = config["path_design"].as<string>();
    auto path_design_m = config["path_design_m"].as<string>();
    auto path_topo = config["path_topo"].as<string>();
    auto path_w2k = config["path_w2k"].as<string>();
    auto path_cloud = config["path_cloud"].as<string>();
    boost::format file_color(path_img + "color_%d.png");
    boost::format file_depth(path_img + "depth_%d.png");
    boost::format file_cloud(path_cloud + "cloud_%s.pcd");
    // marker
    auto num_photos = config["num_photos"].as<int>();
    auto num_markers = config["num_markers"].as<int>();
    auto marker_size = config["marker_size"].as<double>();
    auto indices_list = config["reorder_indices_list"].as< vector<int> >();
    auto corner_refinement = config["corner_refinement"].as<int>();
    auto view_corner = config["view_corner"].as<bool>();
    // segmentation
//    auto seg_param = config["seg_param"].as<double>();
    auto seg_length = config["seg_length"].as<double>();
    auto dis_max = config["dis_max"].as<double>();
    auto seg_thresh = config["seg_thresh"].as<double>();
    // cloud processing
    auto leaf_size = config["leaf_size"].as<float>();
    auto radius = config["radius"].as<float>();
//    auto fit_thresh = config["fit_thresh"].as<float>();
    auto mean_k = config["mean_k"].as<int>();

    /* ===============
     * read mark poses
     * =============== */
    double **data_marker = readData(path_w2k, num_markers, LEN_T_4_4);
    vector<Eigen::Matrix4d> w2k;
    for (int i = 0; i < num_markers; ++i) {
        w2k.emplace_back(Eigen::Map<Eigen::Matrix4d>(data_marker[i]).transpose());
    }
    delete data_marker;

    /* ================
     * read design topo
     * ================ */
    double **data_topo = readData(path_topo, num_poles, LEN_TOPO_VEC);
    if (data_topo[task_index][TOPO_TYPE] == -1.0)
        throw runtime_error("Task invalid!");

    /* ================
     * read design data
     * ================ */
    double **data_design = readData(path_design, num_poles, LEN_CYL);
    double **data_design_m = readData(path_design_m, num_poles, LEN_CYL);
    Cylinder pole;
    double seg_param = -1.0;
    int target_index = -1;
    if (task_type == det::SCAN_MATERIAL) {
        pole = Cylinder(data_design_m[task_index]);
        if (task_node_type == det::NODE_FROM) {
            seg_param = data_topo[task_index][TOPO_FROM_M_PARAM];
            target_index = (int)data_topo[task_index][TOPO_FROM_INDEX];
        } else {
            if (data_topo[task_index][TOPO_TYPE] == 0.0) {
                seg_param = Eigen::Vector3d(pole[4], pole[5], pole[6]).norm()- 0.5 * seg_length;
            } else {
                seg_param = data_topo[task_index][TOPO_TO_M_PARAM];
                target_index = (int)data_topo[task_index][TOPO_TO_INDEX];
            }
        }
    } else if (task_node_type == det::NODE_FROM) {
        pole = Cylinder(data_design[(int)data_topo[task_index][TOPO_FROM_INDEX]]);
        seg_param = data_topo[task_index][TOPO_FROM_B_PARAM];
        target_index = (int)data_topo[task_index][TOPO_FROM_INDEX];
    } else {
        if (data_topo[task_index][TOPO_TYPE] == 0.0)
            throw runtime_error("Why do u scan the target point?");
        else {
            pole = Cylinder(data_design[(int)data_topo[task_index][TOPO_TO_INDEX]]);
            seg_param = data_topo[task_index][TOPO_TO_B_PARAM];
            target_index = (int)data_topo[task_index][TOPO_TO_INDEX];
        }
    }
    delete data_design;
    cout << "pole: \n" << pole << endl;
    cout << "seg_param: " << seg_param << endl;
    cout << "path_result: " << (boost::format(path_result)%target_index).str() << endl;

    /* ============================
     * segment and align the clouds
     * ============================ */
    PointCloudG cloud_full(new pcl::PointCloud<PointG>);
    pcl::PCDWriter writer;
    for (int i = 0; i < num_photos; ++i) {
        Mat color = imread((file_color % i).str());
        Mat depth = imread((file_depth % i).str(), CV_LOAD_IMAGE_UNCHANGED);
        vector< vector<Point2f> > corners;
        vector<int> ids;
        det::detectMarker(color, corners, ids, marker_size, view_corner, CornerRefineMethod(corner_refinement));
        if (view_corner) {
            imshow("color", color);
            waitKey();
        }
        Eigen::Isometry3d c2w = det::solve_pose(corners, ids, marker_size, indices_list, w2k);
        PointCloudG cloud = det::solve_seg(depth, c2w, pole, seg_param, seg_length, dis_max, seg_thresh);
        det::clean_cloud(cloud, mean_k);
        det::downsample_cloud(cloud, leaf_size);
        *cloud_full += *cloud;

        cout << i << ": " << endl;
        cout << "\t number of markers: " << ids.size() << endl;
        cout << "\t c2w: " << c2w.matrix() << endl;
    }
    det::clean_cloud(cloud_full, mean_k);
    det::downsample_cloud(cloud_full, leaf_size);
    cout << "final cloud has " << cloud_full->points.size() << " points." << endl;
    writer.write((file_cloud%"full").str(), *cloud_full, false);

    /* ================
     * cylinder fitting
     * ================ */

//    Cylinder c = det::ransac(cloud_full, radius, dis_threshold);
//    writer.write((file_cloud%"ransac").str(), *cloud_full, false);

    Cylinder c = det::fitting(cloud_full, get_seg(pole, seg_param - 0.5 * seg_length, radius));
    c = det::finite_seg(cloud_full, c);
    cout << "final parameters: \n" << c  << endl;

    /* ================
     * writing to file
     * ================ */
    ofstream out_file;
    out_file.open((boost::format(path_result)%target_index).str());
    for (int i = 0; i < LEN_CYL; ++i) {
        out_file << c[i] << " ";
    }
    out_file.close();

    delete data_design;
    return 0;
}

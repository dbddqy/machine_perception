//
// Created by yue on 30.09.20.
//

#include <yaml-cpp/yaml.h>
#include <boost/format.hpp>

#include <lib_det.hpp>
#include <utility.hpp>

using namespace std;
using namespace cv;
using namespace utility;

int main(int argc, char **argv) {
    /* ===============
     * load parameters
     * =============== */
    YAML::Node config = YAML::LoadFile("../../config/config_det.yml");
    auto task_index = config["task_index"].as<int>();
    auto num_poles = config["num_poles"].as<int>();
    // path
    auto path_topo = config["path_topo"].as<string>();
    auto path_w2k = config["path_w2k"].as<string>();
    auto path_det_result = config["path_det_result"].as<string>();
    auto path_adt_result = config["path_adt_result"].as<string>();
    // marker
    auto num_photos = config["num_photos"].as<int>();
    auto num_markers = config["num_markers"].as<int>();
    auto marker_size = config["marker_size"].as<double>();
    auto indices_list = config["reorder_indices_list"].as< vector<int> >();
    auto corner_refinement = config["corner_refinement"].as<int>();
    auto view_corner = config["view_corner"].as<bool>();

    /* ================
     * read design topo
     * ================ */
    double *data_topo = readData(path_topo, num_poles, LEN_TOPO_VEC)[task_index];
    if (data_topo[TOPO_TYPE] == -1.0)
        throw runtime_error("Task invalid!");

    /* =====================
     * read detection result
     * ===================== */
    double *data_det = readDataRow((boost::format(path_det_result)%data_topo[TOPO_TO_INDEX]).str(), LEN_CYL);

    for (int i = 0; i < LEN_CYL; ++i) {
        cout << data_det[i] << " ";
    }
    cout << endl;
    return 0;
}

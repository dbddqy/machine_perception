//
// Created by yue on 23.09.20.
//

#include <boost/format.hpp>
#include <lib_rs.hpp>
#include <lib_det.hpp>
#include <yaml-cpp/yaml.h>

using namespace cv;
using namespace std;
namespace det = detection;

int main(int argc, char **argv) {
    /* ===============
     * load parameters
     * =============== */
    YAML::Node config = YAML::LoadFile("../../config/config_take_frame.yml");
    auto path_img = config["path_img"].as<string>();
    auto num_saved = config["save_init_index"].as<int>();

    D415 d415;
    d415.print_intrinsics();
    Mat color, depth;
    namedWindow("color", CV_WINDOW_NORMAL);
    cvResizeWindow("color", 960, 540);
    boost::format file_color(path_img + "color_%d.png");
    boost::format file_depth(path_img + "depth_%d.png");
    boost::format file_color_drawn(path_img + "color_drawn_%d.png");
    while (true) {
        char key = waitKey(1);
        if (key == 'q') break;

        d415.receive_frame(color, depth);
        Mat color_drawn = color.clone();
        vector< vector<Point2f> > corners;
        vector<int> ids;
        det::detectMarker(color_drawn, corners, ids, 0.07, true, cv::aruco::CORNER_REFINE_NONE);
        imshow("color", color_drawn);

        if (key == 's') {
            imwrite((file_color % num_saved).str(), color);
            imwrite((file_depth % num_saved).str(), depth);
            imwrite((file_color_drawn % num_saved).str(), color_drawn);
            cout << "frame " << num_saved << " saved!" << endl;
            ++ num_saved;
        }
    }

    return 0;
}
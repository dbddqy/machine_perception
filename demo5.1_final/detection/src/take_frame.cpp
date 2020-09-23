//
// Created by yue on 23.09.20.
//

#include <boost/format.hpp>
#include <lib_rs.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    D415 d415;
    Mat color, depth;
    int num_saved = 0;
    boost::format file_color("../../data2D/color_%d.png");
    boost::format file_depth("../../data2D/depth_%d.png");
    while (true) {
        char key = waitKey(1);
        if (key == 'q') break;

        d415.receive_frame(color, depth);
        imshow("color", color);

        if (key == 's') {
            imwrite((file_color % num_saved).str(), color);
            imwrite((file_depth % num_saved).str(), depth);
            ++ num_saved;
        }
    }
}
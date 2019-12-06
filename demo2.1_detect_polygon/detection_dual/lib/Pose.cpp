//
// Created by 戚越 on 2019-12-05.
//

#include <opencv2/opencv.hpp>

using namespace cv;

class Pose {
public:
    Mat m;
    Mat R() { return m(Rect(0, 0, 3, 3)); }
    Mat t() { return m(Rect(3, 3, 1, 3)); }
};
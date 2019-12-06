#include "main.hpp"

int main() {
    VideoCapture inputVideo("../../data/02.mov");
//    Point3d a(1., 0., 0.), b(0, 1, 0), c(-1, -3, 0);
//    vector<Point3d> polyline;
//    polyline.push_back(a);
//    polyline.push_back(b);
//    polyline.push_back(c);
//    Vec3d v(1, 2, 3);
//    cout << v[-1];
    // load camera parameters
    while (true) {
        Mat img;
        inputVideo >> img;
        if (img.empty()) break;
        Rect rectL(0, 0, 640, 480);
        Rect rectR(640, 0, 640, 480);
        Mat imgL = img(rectL), imgR = img(rectR);
        remap(imgL, imgL, c.mapL1, c.mapL2, CV_INTER_LINEAR);
        remap(imgR, imgR, c.mapR1, c.mapR2, CV_INTER_LINEAR);
        // left
        vector< vector<Point> > contoursL = getContours(imgL);
        drawContours(imgL, contoursL, -1, Scalar(0, 0, 255));
        // right
        vector< vector<Point> > contoursR = getContours(imgR);
        drawContours(imgR, contoursR, -1, Scalar(0, 0, 255));
        // matching
        vector< vector<Point3d> > polylines;
        if (validate(contoursL, contoursR, polylines))
            for (auto polyline : polylines) {
                cout << "from pose" << getPose(polyline).t() << endl;
                cout << "from center" << getCenter(polyline) << endl;
            }
        // show images
        imshow("rawL", imgL);
        imshow("rawR", imgR);

        char key = (char) waitKey(1);
        if (key == 27)
            break;
    }
    return 0;
}

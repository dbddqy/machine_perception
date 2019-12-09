#include "main.hpp"

int main() {
    VideoCapture inputVideo("../../data/02.mov");

    // load camera parameters
    while (true) {
        Mat img;
        inputVideo >> img;
        if (img.empty()) break;
        Rect rectL(0, 0, 640, 480);
        Rect rectR(640, 0, 640, 480);
        Mat imgL = img(rectL), imgR = img(rectR);
        remap(imgL, imgL, C.mapL1, C.mapL2, CV_INTER_LINEAR);
        remap(imgR, imgR, C.mapR1, C.mapR2, CV_INTER_LINEAR);
        // left
        vector< vector<Point> > contoursL = getContours(imgL);
        drawContours(imgL, contoursL, -1, Scalar(0, 0, 255));
//        // log point order
//        for (auto c : contoursL)
//            for (int i = 0; i < c.size(); ++i) {
//                putText(imgL, to_string(i), c[i], FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2);
//            }

        // right
        vector< vector<Point> > contoursR = getContours(imgR);
        drawContours(imgR, contoursR, -1, Scalar(0, 0, 255));
        // matching
        vector< vector<Point3d> > polylines;
        if (validate(contoursL, contoursR, polylines))
            for (auto polyline : polylines) {
                Pose pose = getPose(polyline);
                int index = match(polyline, pose);
//                cout << "from pose" << pose.origin().t() << " index:" << index << endl;
                if (index != -1) {
                    imgL = drawPose(imgL, pose, index, 15);
                }
            }
        // show images
        imshow("rawL", imgL);
        imshow("rawR", imgR);

        char key = (char) waitKey();
        if (key == 27)
            break;
    }
    return 0;
}

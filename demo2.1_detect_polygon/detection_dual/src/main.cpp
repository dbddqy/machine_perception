#include "main.hpp"

int ITERATION_COUNT = 0;

int main() {
    VideoCapture inputVideo("../../data/02.mov");
//    VideoWriter writer;
//    writer.open("result.avi", CV_FOURCC('M', 'J', 'P', 'G'), 36, Size(1280, 480), true);

    // for measuring running time
    struct timespec tpStart{};
    struct timespec tpEnd{};
    clock_gettime(CLOCK_MONOTONIC, &tpStart);

    while (true) {
//        if(ITERATION_COUNT == 301) break;
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
        // right
        vector< vector<Point> > contoursR = getContours(imgR);
        drawContours(imgR, contoursR, -1, Scalar(0, 0, 255));
        // matching
        vector< vector<Point3d> > polylines;
        if (validate(contoursL, contoursR, polylines))
            for (auto polyline : polylines) {
                sortPolyline(polyline);
                Pose pose = getPose(polyline);
                int index = match(polyline, pose);
                if (index != -1) {
                    imgL = drawPose(imgL, pose, index, 15);
                    cout << "piece found: " << pose.origin().t() << " index:" << index << endl;
                }
            }
        // show images
//        imshow("rawL", imgL);
//        imshow("rawR", imgR);
//        imwrite("rawL.png", imgL);
//        imwrite("rawR.png", imgR);
//        Mat output(480, 1280, img.type());
//        imgL.copyTo(output(Rect(0, 0, 640, 480)));
//        imgR.copyTo(output(Rect(640, 0, 640, 480)));
//        imshow("output", output);
//        writer.write(output);

//        char key = (char) waitKey(1);
//        if (key == 27)
//            break;
        ITERATION_COUNT += 1;
        cout << ITERATION_COUNT << endl;
    }
    // for measuring running time
    clock_gettime(CLOCK_MONOTONIC, &tpEnd);
    long timedif = 1000000*(tpEnd.tv_sec - tpStart.tv_sec) + (tpEnd.tv_nsec - tpStart.tv_nsec) / 1000;
    fprintf(stdout, "it took %ld microseconds\n", timedif);

    return 0;
}

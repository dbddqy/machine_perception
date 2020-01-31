//
// Created by yue on 26.01.20.
//

#include <iostream>
#include <boost/format.hpp>  // for formating strings
#include <kinect.hpp>

using namespace std;
using namespace cv;

int main() {
    kinectInit();
    int count = 0;
    while (waitKey(1) != 27) {
        getFrame();
        imshow("bgrd", *matBGRD);
        imshow("depth", *matDepth / 4500.0f);
        if (waitKey(1) == 'q') {
            imwrite((boost::format("../../data/color/color_%d.png")%count).str(), *matBGRD);
            Mat depth = matDepth->clone();
            Mat matDepth16 = Mat::zeros(depth.size(), CV_16UC1);
            depth.convertTo(matDepth16, CV_16UC1);
            imwrite((boost::format("../../data/depth/depth_%d.png")%count).str(), matDepth16);

            pcl::io::savePCDFileBinary((boost::format("../../data/point_cloud/cloud_%d.pcd")%count).str(), *getPointCloud());
            count += 1;
        }
    }
    kinectClose();
    return 0;
}


//
// Created by yue on 30.01.20.
//

#include <iostream>
#include <boost/format.hpp>  // for formating strings

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

using namespace std;
using namespace cv;

class CameraConfig {
public:
    Mat M;
    Mat distortion;
    Mat MInv;
    double fx, fy, cx, cy;

    CameraConfig() {
        M = (Mat_<double>(3, 3)
                << 1382.23, 0., 953.567
                , 0., 1379.46, 532.635
                , 0., 0., 1.);
        distortion = (Mat_<double >(1, 5)
                << 0.0, 0.0, 0.0, 0.0, 0.0);
        MInv = M.inv();
        fx = 1382.23; fy = 1379.46; cx = 953.567; cy = 532.635;
    };
};

CameraConfig C;

PointCloudG points2pcl(const rs2::points& points) {
    PointCloudG cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points) {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
    return cloud;
}

int main(int argc, char **argv) {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1920, 1080, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);

    rs2::pointcloud pc;
    rs2::points points;

    namedWindow("color", 0);
//    namedWindow("with mask", 0);
//    namedWindow("01", 0);
    cvResizeWindow("color", 960, 540);
//    cvResizeWindow("with mask", 960, 540);
//    cvResizeWindow("01", 640, 360);

    int num_pc_saved = 0;

    // start
    while (waitKey(1) != 'q') {
        rs2::frameset frameset = pipe.wait_for_frames();
        frameset = align_to_color.process(frameset);

        auto depth = frameset.get_depth_frame();
        auto color = frameset.get_color_frame();

        Mat matColor(Size(1920, 1080), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
        Mat matDepth(Size(1920, 1080), CV_16U, (void*)depth.get_data(), Mat::AUTO_STEP);

        imshow("color", matColor);

//        Mat mask = Mat::zeros(matDepth.size(), CV_8UC1);
//        for (int i = 0; i < matColor.rows; ++i) {
//            uint16_t *p = matDepth.ptr<uint16_t>(i);
//            for (int j = 0; j < matColor.cols; ++j) {
//                if (p[j] > 700 && p[j] < 900)
//                    mask.at<uchar>(i, j) = 255;
//            }
//        }
//        Mat colorWithMask;
//        matColor.copyTo(colorWithMask, mask);
//        imshow("with mask", colorWithMask);

        // point cloud
        if (waitKey(1) == 's') {
            pc.map_to(color);
            points = pc.calculate(depth);

            PointCloudG pclPC = points2pcl(points);
            boost::format fmt( "../test_data/data/cloud_full0%d.pcd" );
            pcl::io::savePCDFileBinary((fmt%num_pc_saved).str(), *pclPC);
            cout << "PointCloud has: " << pclPC->points.size() << " data points." << endl;
            num_pc_saved += 1;
        }
    }

    return 0;
}
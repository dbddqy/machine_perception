//
// Created by yue on 26.01.20.
//

#ifndef POINTCLOUD_HPP
#define POINTCLOUD_HPP

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <librealsense2/hpp/rs_frame.hpp>
#include <opencv2/opencv.hpp>

//cv::Mat;

using namespace std;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

PointCloudG points2pcl(const rs2::points& points);

PointCloudG mat2pcl(const cv::Mat& depth, const cv::Mat& mask);

void detectCylinderRANSAC(const PointCloudG& pc);

int test();

#endif //POINTCLOUD_HPP

//
// Created by yue on 26.01.20.
//

#ifndef KINECT_HPP
#define KINECT_HPP

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// for kinect v2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

using namespace std;
using namespace libfreenect2;
using namespace cv;

// kinect
Freenect2 freenect2;
Freenect2Device *dev = 0;
SyncMultiFrameListener *listener;
FrameMap frames;
Registration* registration;
Frame undistorted(512, 424, 4), registered(512, 424, 4);

// frame from kinect
Mat *matDepth, *matBGRD;

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<PointC> PointCloud;

int kinectInit();

void kinectClose();

int getFrame();

PointCloud::Ptr getPointCloud();

#endif //KINECT_HPP

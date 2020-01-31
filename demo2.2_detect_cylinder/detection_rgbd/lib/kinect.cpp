//
// Created by yue on 26.01.20.
//

#include "kinect.hpp"

int kinectInit() {
    if(freenect2.enumerateDevices() == 0) {
        cout << "no device connected!" << endl;
        return -1;
    }
    string serial = freenect2.getDefaultDeviceSerialNumber();
    dev = freenect2.openDevice(serial);
    int types = 0;
    types |= libfreenect2::Frame::Color;
    types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    listener = new SyncMultiFrameListener(types);
    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    if (!dev->start())
        return -1;
    registration = new Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    return 0;
}

void kinectClose() {
    dev->stop();
    dev->close();
}

int getFrame() {
    if (!listener->waitForNewFrame(frames, 10*1000)) { // 10 sconds
        cout << "timeout!" << endl;
        return -1;
    }
    Frame *rgb = frames[Frame::Color];
    Frame *depth = frames[Frame::Depth];
    registration->apply(rgb, depth, &undistorted, &registered, true);

    matDepth = new Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
    matBGRD = new Mat(registered.height, registered.width, CV_8UC4, registered.data);
    listener->release(frames);
}

PointCloud::Ptr getPointCloud() {
    PointCloud::Ptr pointCloud(new PointCloud);
    for (int v = 0; v < 424; v++)
        for (int u = 0; u < 512; u++) {
            float x, y, z, color;
            registration->getPointXYZRGB(&undistorted, &registered, v, u, x, y, z, color);
            const uint8_t *color_3b = reinterpret_cast<uint8_t*>(&color);
            PointC p ;
            p.x = x;
            p.y = y;
            p.z = z;
            p.b = color_3b[0];
            p.g = color_3b[1];
            p.r = color_3b[2];
            pointCloud->points.push_back( p );
        }
    pointCloud->is_dense = false;

    return pointCloud;
}

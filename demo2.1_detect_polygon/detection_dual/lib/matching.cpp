//
// Created by yue on 04.12.19.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <CameraConfig.cpp>
#include <Pose.cpp>
#include <math.h>

using namespace std;
using namespace cv;

bool validate(vector< vector<Point> > contoursL, vector< vector<Point> > contoursR, vector< vector<Point3d> > &realPolylines) {
    if (contoursL.size() != contoursR.size()) return false;

    for (int i = 0; i < contoursL.size(); ++i) {
        if (contoursL[i].size() != contoursR[i].size()) return false;
        if (contoursL[i].size() < 3) return false;

        vector<Point3d> polylineToAppend;
        for (int j = 0; j < contoursL[i].size(); ++j) {
            double depth = c.b * c.f / (contoursL[i][j].x - contoursR[i][j].x);
//            cout << depth << endl;
            // position L
            Mat positionLPixel = (Mat_<double>(3, 1)
                    << contoursL[i][j].x*depth, contoursL[i][j].y*depth, depth);
            Mat positionL = c.cameraLInv * positionLPixel;
//            cout << "positionL: " << Point3d(positionL) << endl;
            // position R
            Mat positionRPixel = (Mat_<double>(3, 1)
                    << contoursR[i][j].x*depth, contoursR[i][j].y*depth, depth);
            Mat positionR = c.cameraRInv * positionRPixel;
//            cout << "positionR: " << Point3d(positionR) << endl;
            if (abs(sum(positionL-positionR+c.T)[0]) > 5) return false;
            polylineToAppend.push_back(Point3d(positionL));
        }
//        cout << "contour left: " << contoursL[i] << endl;
//        cout << "contour right: " << contoursR[i] << endl;
        realPolylines.push_back(polylineToAppend);
    }

    return true;
}

Point3d getCenter(vector<Point3d> polyline) {
    Point3d center(0., 0., 0.);
    for (auto p : polyline) center += p;
    return center;
}

Vec3d getNormal(vector<Point3d> polyline) {
    Point3d center = getCenter(polyline);
    Vec3d v0 = polyline[0];
    Vec3d v1 = polyline[1];
    return v0.cross(v1);
}

Pose getPose(vector<Point3d> polyline) {
    Point3d center = getCenter(polyline);
    for (auto p : polyline) {
        Vec3d vt = p - center;
    }
}
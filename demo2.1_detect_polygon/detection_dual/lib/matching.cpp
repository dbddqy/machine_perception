//
// Created by yue on 04.12.19.
//

#include <matching.hpp>

bool validate(vector<vector<Point> > contoursL, vector<vector<Point> > contoursR, vector<vector<Point3d> > &realPolylines) {
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
    center.x /= polyline.size();
    center.y /= polyline.size();
    center.z /= polyline.size();
    return center;
}

Vec3d getNormal(vector<Point3d> polyline) {
    Point3d center = getCenter(polyline);
    Vec3d v0 = polyline[0] - center;
    Vec3d v1 = polyline[1] - center;
    return unitize(v0.cross(v1));
}

Pose getPose(vector<Point3d> polyline) {
    Vec3d zAxis = unitize(getNormal(polyline));
    double maxDis = 0.0;
    Vec3d xAxis; // choose farthest vec as x-axis
    Point3d center = getCenter(polyline);
    for (auto p : polyline) {
        Vec3d vt = p - center;
        if (getLength(vt) > maxDis) {
            maxDis = getLength(vt);
            xAxis = vt;
        }
    }
    Pose pose(center, xAxis, zAxis);
    return pose;
}

Vec3d unitize(Vec3d v) {
    Vec3d unitVec;
    normalize(v, unitVec, 1, 0, NORM_L2);
    return unitVec;
}

double getLength(Vec3d v) {
    return sqrt(v.dot(v));
}


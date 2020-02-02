//
// Created by 戚越 on 2019-12-05.
//

#include <pose.hpp>

pose::pose(Point3d origin, Vec3d rotationVec) {
    Mat r;
    Rodrigues(rotationVec, r);
    Mat t = (Mat_<double>(3, 1) << origin.x, origin.y, origin.z);
    hconcat(r, t, m);
    Mat down = (Mat_<double>(1, 4) << 0., 0., 0., 1.);
    vconcat(m, down, m);
}

//pose::pose(Vec3d origin, Vec3d rotationVec) {
//    Point3d point(origin);
//    pose(point, rotationVec);
//}

pose::pose(Point3d origin, Vec3d unitX, Vec3d unitZ) {
    Vec3d unitY = unitZ.cross(unitX);
    m = (Mat_<double>(4, 4) <<
            unitX[0], unitY[0], unitZ[0], origin.x,
            unitX[1], unitY[1], unitZ[1], origin.y,
            unitX[2], unitY[2], unitZ[2], origin.z,
            0., 0., 0., 1.
            );
}

Mat pose::R() { return m(Rect(0, 0, 3, 3)); }

Mat pose::origin() { return m(Rect(3, 0, 1, 3)); }

Mat pose::xAxis() { return m(Rect(0, 0, 1, 3)); }

Mat pose::yAxis() { return m(Rect(1, 0, 1, 3)); }

Mat pose::zAxis() { return m(Rect(2, 0, 1, 3)); }

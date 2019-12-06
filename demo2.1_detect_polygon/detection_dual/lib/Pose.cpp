//
// Created by 戚越 on 2019-12-05.
//

#include <Pose.hpp>

Pose::Pose(Point3d origin, Vec3d rotationVec) {
    Mat r;
    Rodrigues(rotationVec, r);
    Mat t = (Mat_<double>(3, 1) << origin.x, origin.y, origin.z);
    hconcat(r, t, m);
    Mat down = (Mat_<double>(1, 4) << 0., 0., 0., 1.);
    vconcat(m, down, m);
}

Pose::Pose(Point3d origin, Vec3d unitZ, Vec3d unitX) {
    Vec3d unitY = unitZ.cross(unitX);
    m = (Mat_<double>(4, 4) <<
            unitX[0], unitY[0], unitZ[0], origin.x,
            unitX[1], unitY[1], unitZ[1], origin.y,
            unitX[2], unitY[2], unitZ[2], origin.z,
            0., 0., 0., 1.
            );
}

Mat Pose::R() { return m(Rect(0, 0, 3, 3)); }

Mat Pose::t() { return m(Rect(3, 3, 1, 3)); }

//
// Created by yue on 26.09.20.
//

#ifndef DETECTION_UTILITY_HPP
#define DETECTION_UTILITY_HPP

#include <Eigen/Core>
#include <string>

#define LEN_CYL 7
typedef Eigen::Matrix<double, LEN_CYL, 1> Cylinder;

namespace utility {

    int findIndex(std::vector<int> indices, int index);

    Eigen::Vector4d getCorner(int index, double size);

    double** readData(std::string path, int row, int col);

    double* readDataRow(std::string path, int col);

    double distance2cylinder(Cylinder pole, Eigen::Vector3d point,
            double start_param, double end_param, bool is_finite=true);

    double line_closest_param(Eigen::Vector3d start, Eigen::Vector3d dir, Eigen::Vector3d point);

    Cylinder get_seg(Cylinder pole, double start_param, double r);
}


#endif //DETECTION_UTILITY_HPP

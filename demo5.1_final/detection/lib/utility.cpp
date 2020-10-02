//
// Created by yue on 26.09.20.
//

#include "utility.hpp"
#include <fstream>

using namespace std;

int utility::findIndex(vector<int> indices, int index) {
    auto iElement = find(indices.begin(), indices.end(), index);
    if( iElement != indices.end() ) {
        return distance(indices.begin(),iElement);
    } else {
        return -1;
    }
}

Eigen::Vector4d utility::getCorner(int index, double size) {
    switch (index) {
        case 0: return {-0.5*size, 0.5*size, 0.0, 1.0};
        case 1: return {0.5*size, 0.5*size, 0.0, 1.0};
        case 2: return {0.5*size, -0.5*size, 0.0, 1.0};
        case 3: return {-0.5*size, -0.5*size, 0.0, 1.0};
        default: return Eigen::Vector4d::Zero();
    }
}

double* utility::readDataRow(std::string path, int col) {
    ifstream file;
    file.open(path, ios::in);
    auto *data = new double[col];
    for (int i = 0; i < col; ++i)
        file >> data[i];
    file.close();
    return data;
}

double** utility::readData(string path, int row, int col) {
    ifstream file;
    file.open(path, ios::in);
    auto **data = new double*[row];
    for (int i = 0; i < row; ++i) {
        data[i] = new double[col];
        for (int j = 0; j < col; ++j)
            file >> data[i][j];
    }
    file.close();
    return data;
}

double utility::distance2cylinder(Cylinder pole, Eigen::Vector3d point,
                                  double start_param, double end_param, bool is_finite) {
    Eigen::Vector3d start(pole[1], pole[2], pole[3]);
    Eigen::Vector3d dir(pole[4], pole[5], pole[6]);
    dir /= dir.norm();
    Eigen::Vector3d seg_start = start + start_param * dir;
    Eigen::Vector3d seg_end = start + end_param * dir;
    if (is_finite) {
        if (line_closest_param(seg_start, dir, point) < 0) return 0.0; // 0: not in the segment
        if (line_closest_param(seg_end, -dir, point) < 0) return 0.0; // 0: not in the segment
    }
    return sqrt((point-seg_start).dot((point-seg_start))-pow((point-seg_start).dot(dir), 2)) - pole[0];
}

Cylinder utility::get_seg(Cylinder pole, double start_param, double r) {
    start_param /= Eigen::Vector3d(pole[4], pole[5], pole[6]).norm();
    double temp[LEN_CYL] = {r, pole[1]+start_param*pole[4], pole[2]+start_param*pole[5], pole[3]+start_param*pole[6],
                            pole[4], pole[5], pole[6]};
    return Cylinder(temp);
}

double utility::line_closest_param(Eigen::Vector3d start, Eigen::Vector3d dir, Eigen::Vector3d point) {
    return (point-start).dot(dir) / dir.norm();
}

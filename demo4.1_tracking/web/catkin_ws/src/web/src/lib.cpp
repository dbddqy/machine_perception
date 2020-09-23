#include "lib.h"

CameraConfig::CameraConfig() {
    M = (Mat_<double>(3, 3)
            << 1382.23, 0., 953.567
            , 0., 1379.46, 532.635
            , 0., 0., 1.);
    distortion = (Mat_<double >(1, 5)
            << 0.0, 0.0, 0.0, 0.0, 0.0);
    MExt = (Mat_<double>(3, 4)
            << 1382.23, 0., 953.567, 0.
            , 0., 1379.46, 532.635, 0.
            , 0., 0., 1., 0.);
    MInv = M.inv();
    fx = 1382.23; fy = 1379.46; cx = 953.567; cy = 532.635;
}

bool checkState(int f_state) {
    int state;
    ros::param::get("f_state", state);
    return (state == f_state);
}

double **readData(string path, int row, int col) {
    ifstream file;
    file.open(path, ios::in);
    double **data = new double*[row];
    for (int i = 0; i < row; ++i) {
        data[i] = new double[col];
        for (int j = 0; j < col; ++j)
            file >> data[i][j];
    }
    file.close();
    return data;
}

int findIndex(vector<int> indices, int index) {
    vector <int>::iterator iElement = find(indices.begin(), indices.end(), index);
    if( iElement != indices.end() ) {
        return distance(indices.begin(),iElement);
    } else {
        return -1;
    }
}

Eigen::Vector4d getCorner(int index, double size) {
    switch (index) {
        case 0: return Eigen::Vector4d(-0.5*size, 0.5*size, 0.0, 1.0);
        case 1: return Eigen::Vector4d(0.5*size, 0.5*size, 0.0, 1.0);
        case 2: return Eigen::Vector4d(0.5*size, -0.5*size, 0.0, 1.0);
        case 3: return Eigen::Vector4d(-0.5*size, -0.5*size, 0.0, 1.0);
    }
}

Mat readPose() {
    vector<double> data_c2w;
    ros::param::get("c2w", data_c2w);
    Vec3d rvec(data_c2w[0], data_c2w[1], data_c2w[2]);
    Vec3d tvec(data_c2w[3], data_c2w[4], data_c2w[5]);
    Mat r, c2w;
    Rodrigues(rvec, r);
    hconcat(r, tvec*1000.0, c2w);
    Mat down = (Mat_<double>(1, 4) << 0., 0., 0., 1.);
    vconcat(c2w, down, c2w);
    return c2w;
}

double dis(Vec3d p1, Vec3d p2) {
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]));
}

double distance2cylinder(double *cylinder_param, Vec3d pt, double start_param, double end_param) {
    Vec3d start(cylinder_param[1], cylinder_param[2], cylinder_param[3]);
    Vec3d dir(cylinder_param[4], cylinder_param[5], cylinder_param[6]);
    Vec3d p_start = start + start_param * dir;
    Vec3d p_end = start + end_param * dir;
    if ((pt-p_start).dot(dir) < 0) return 0; // 0: not in the segment
    if ((p_end-pt).dot(dir) < 0) return 0; // 0: not in the segment
    return sqrt((pt-p_start).dot(pt-p_start)-pow((pt-p_start).dot(dir), 2)) - cylinder_param[0];
}
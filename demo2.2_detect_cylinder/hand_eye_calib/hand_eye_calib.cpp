#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>
#include <boost/format.hpp>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

using namespace std;

// 代价函数的计算模型
struct HAND_EYE_COST {
    HAND_EYE_COST(Eigen::Matrix4d x1, Eigen::Matrix4d x2) : _x1(x1), _x2(x2){}

    // 残差的计算
    template<typename T>
    bool operator()(
            const T *const translation, // 3 dim
            const T *const angleAxis, // 3 dim
            T *residual) const {
        T R[9];
        ceres::AngleAxisToRotationMatrix(angleAxis, R);
        Eigen::Matrix<T, 4, 4> tran;
        tran << R[0], R[3], R[6], translation[0],
                R[1], R[4], R[7], translation[1],
                R[2], R[5], R[8], translation[2],
                T(0.0), T(0.0), T(0.0), T(1.0);
        Eigen::Matrix<T, 4, 4> _x1_Matrix = _x1.cast<T>();
        Eigen::Matrix<T, 4, 4> _x2_Matrix = _x2.cast<T>();
        Eigen::Matrix<T, 4, 4> Cost_Matrix = _x1_Matrix * tran - tran * _x2_Matrix;
//        residual[0] = (Cost_Matrix.array()*Cost_Matrix.array()).sum();
        residual[0] = Cost_Matrix(0, 0); residual[1] = Cost_Matrix(0, 1); residual[2] = Cost_Matrix(0, 2); residual[3] = Cost_Matrix(0, 3);
        residual[4] = Cost_Matrix(1, 0); residual[5] = Cost_Matrix(1, 1); residual[6] = Cost_Matrix(1, 2); residual[7] = Cost_Matrix(1, 3);
        residual[8] = Cost_Matrix(2, 0); residual[9] = Cost_Matrix(2, 1); residual[10] = Cost_Matrix(2, 2); residual[11] = Cost_Matrix(2, 3);
        residual[12] = Cost_Matrix(3, 0); residual[13] = Cost_Matrix(3, 1); residual[14] = Cost_Matrix(3, 2); residual[15] = Cost_Matrix(3, 3);
        return true;
    }

    const Eigen::Matrix4d _x1, _x2;    // x数据
};

void readData(char *path, int n, vector<Eigen::Matrix4d> &As, vector<Eigen::Matrix4d> &Bs) {
    ifstream file;
    file.open(path, ios::in);
    vector<Eigen::Matrix4d> tran_list_w2e, tran_list_c2o;
    for (int i = 0; i < n; ++i) {
        double data[16] = {0};
        for (auto& d : data)
            file >> d;
        tran_list_w2e.emplace_back(Eigen::Map<Eigen::Matrix4d>(data).transpose());
        for (auto& d : data)
            file >> d;
        tran_list_c2o.emplace_back(Eigen::Map<Eigen::Matrix4d>(data).transpose());
    }
    file.close();
    for (int i = 0; i < n-1; ++i) {
        As.push_back(tran_list_w2e[i+1].inverse()*tran_list_w2e[i]);
        Bs.push_back(tran_list_c2o[i+1]*tran_list_c2o[i].inverse());
    }
}

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "please enter the file path" << endl;
        return -1;
    }

    double t0 = 0.0, t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0, t5 = 1.57;        // 估计参数值
    double translation[3] = {t0, t1, t2};
    double angleAxis[3] = {t3, t4, t5};

    // 5 pairs of data
    int N = 5;
    vector<Eigen::Matrix4d> As, Bs; // As: Twe2.inv()*Twe1  Bs: Tco2*Tco1.inv()
    readData(argv[1], N, As, Bs);
    // "../fake_data/fake_data.txt"

    // 构建最小二乘问题
    ceres::Problem problem;
    for (int i = 0; i < N-1; i++) {
        problem.AddResidualBlock(     // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<HAND_EYE_COST, 16, 3, 3>(
                        new HAND_EYE_COST(As[i], Bs[i])
                ),
                nullptr,            // 核函数，这里不使用，为空
                translation, angleAxis            // 待估计参数
        );
    }

    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t_start = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary);  // 开始优化
    chrono::steady_clock::time_point t_end = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t_end - t_start);
    cout << "solve time cost = " << time_used.count() << " seconds. " << endl;

    // 输出结果
    cout << summary.BriefReport() << endl;
    cout << "estimated translation = ";
    for (auto a:translation) cout << a << " ";
    cout << endl;

    double R[9];
    ceres::AngleAxisToRotationMatrix<double>(angleAxis, R);
    cout << "estimated R = ";
    for (auto a:R) cout << a << " ";
    cout << endl;

    return 0;
}
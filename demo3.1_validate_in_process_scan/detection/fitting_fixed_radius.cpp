//
// Created by yue on 31.01.20.
//

#include <iostream>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ceres/ceres.h>

using namespace std;

// 代价函数的计算模型
struct CYLINDER_FITTING_COST {
    CYLINDER_FITTING_COST(double x1, double x2, double x3, double r) : _x1(x1), _x2(x2), _x3(x3), _r(r){}

    // 残差的计算
    template<typename T>
    bool operator()(
            const T *const theta, // 模型参数，有6维
            T *residual) const {
        residual[0] = ceres::sqrt((T(_x1)-theta[0])*(T(_x1)-theta[0])
               + (T(_x2)-theta[1])*(T(_x2)-theta[1])
               + (T(_x3)-theta[2])*(T(_x3)-theta[2])
               - ceres::pow((T(_x1)-theta[0])*theta[3]
               + (T(_x2)-theta[1])*theta[4]
               + (T(_x3)-theta[2])*theta[5], 2) / (theta[3]*theta[3]+theta[4]*theta[4]+theta[5]*theta[5])) - T(_r); // r-sqrt(...)
        return true;
    }

    const double _x1, _x2, _x3, _r;    // x数据
};

int main(int argc, char **argv) {
//    double t1 = 0.019792, t2 = 0.0742702, t3 = 0.113163, t4 = 0.416911, t5 = -0.478734, t6 = 0.859715, t7 = 0.178054;        // 估计参数值
    if (argc != 9) {
        cout << "please enter the file path, radius and 6 initial cylinder parameters." << endl;
        return -1;
    }
    double t1=stod(argv[3]), t2=stod(argv[4]), t3=stod(argv[5]), t4=stod(argv[6])
            , t5=stod(argv[7]), t6=stod(argv[8]), r=stod(argv[2]);

    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read (argv[1], *cloud);

    int N = cloud->points.size ();                // 数据点

    vector<double> x1_data, x2_data, x3_data;      // 数据
    for (int i = 0; i < N; i++) {
        x1_data.push_back(cloud->points[i].x);
        x2_data.push_back(cloud->points[i].y);
        x3_data.push_back(cloud->points[i].z);
    }

    double theta[6] = {t1, t2, t3, t4, t5, t6};

    // 构建最小二乘问题
    ceres::Problem problem;
    for (int i = 0; i < N; i++) {
        problem.AddResidualBlock(     // 向问题中添加误差项
                // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                new ceres::AutoDiffCostFunction<CYLINDER_FITTING_COST, 1, 6>(
                        new CYLINDER_FITTING_COST(x1_data[i], x2_data[i], x3_data[i], r)
                ),
                nullptr,            // 核函数，这里不使用，为空
                theta                 // 待估计参数
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
    cout << "estimated theta = ";
    for (auto a:theta) cout << a << " ";
    cout << endl;

    return 0;
}
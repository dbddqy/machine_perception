//
// Created by yue on 23.09.20.
//

#include "header/lib_det.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <ceres/ceres.h>
// for ransac
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

using namespace std;
using namespace cv;
using namespace utility;
namespace C = CameraConfig;

PointCloudG detection::solve_seg(cv::Mat depth, Eigen::Isometry3d c2w, Cylinder pole,
        double seg_param, double seg_length, double dis_max, double dis_thresh) {
    PointCloudG cloud(new pcl::PointCloud<PointG>);
    Eigen::Isometry3d w2c = c2w.inverse();
    for (int v = 0; v < depth.rows; ++v) {
        for (int u = 0; u < depth.cols; ++u) {
            double d = (double)depth.ptr<uint16_t>(v)[u] * 1e-4; // unit: 0.1mm -> m
            if (d == 0.0 || d > dis_max) continue;
            Eigen::Vector4d p_c(((double)u - C::ppx)*d/C::fx, ((double)v - C::ppy)*d/C::fy, d, 1.0);
            Eigen::Vector3d p_w = (w2c * p_c).head<3>();
            double dis2cylinder = distance2cylinder(pole, p_w, seg_param-0.5*seg_length, seg_param+0.5*seg_length);
            if (dis2cylinder != 0.0 && abs(dis2cylinder) < dis_thresh)
                cloud->points.push_back(PointG(p_w[0], p_w[1], p_w[2]));
        }
    }
    return cloud;
}

bool detection::detectMarker(cv::Mat img, vector< vector<cv::Point2f> > &corners, vector<int> &ids, double size,
        bool draw, cv::aruco::CornerRefineMethod corner_refinement) {
    vector<vector<cv::Point2f>> rejectedCandidates;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    parameters->cornerRefinementMethod = corner_refinement;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    cv::aruco::detectMarkers(img, dictionary, corners, ids, parameters, rejectedCandidates);
    if (ids.size() == 0) return false;
    if (!draw) return true;

    cv::aruco::drawDetectedMarkers(img, corners, ids);
    vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(corners, size, C::M_Mat, C::distortion_Mat, rvecs, tvecs);
    for (int i = 0; i < rvecs.size(); ++i) {
        auto rvec = rvecs[i];
        auto tvec = tvecs[i];
        cv::aruco::drawAxis(img, C::M_Mat, C::distortion_Mat, rvec, tvec, size);
    }
    return true;
}

Eigen::Isometry3d detection::solve_pose(vector<vector<cv::Point2f> > corners, vector<int> ids, double size,
        vector<int> indices, vector<Eigen::Matrix4d> marker_poses) {
    vector<Point3f> p_3d;
    vector<Point2f> p_2d;
    for (int i = 0; i < ids.size(); ++i) {
        int index = findIndex(indices, ids[i]);
        if (index == -1) continue;
        for (int k = 0; k < 4; k++) {
            Eigen::Vector4d pt= marker_poses[index] * getCorner(k, size);
            p_3d.push_back(Point3d(pt[0], pt[1], pt[2]));
            p_2d.push_back(corners[i][k]);
        }
    }
    Mat rvec_Mat, tvec_Mat;
    Mat p_2d_Mat = Mat(p_2d).reshape(1);
    Mat p_3d_Mat = Mat(p_3d).reshape(1);
    solvePnP(p_3d_Mat, p_2d_Mat, C::M_Mat, C::distortion_Mat, rvec_Mat, tvec_Mat);
    Eigen::Vector3d rvec, tvec;
    cv2eigen(rvec_Mat, rvec);
    cv2eigen(tvec_Mat, tvec);
    Eigen::Isometry3d c2w = Eigen::Isometry3d::Identity();
    c2w.rotate( Eigen::AngleAxisd(rvec.norm(), rvec / rvec.norm() ) );
    c2w.pretranslate(tvec);
    return c2w;
}

void detection::clean_cloud(PointCloudG cloud, int mean_k) {
    pcl::StatisticalOutlierRemoval<PointG> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (mean_k);
    sor.setStddevMulThresh (0.5);
    sor.filter (*cloud);
}

void detection::downsample_cloud(PointCloudG cloud, float size) {
    pcl::VoxelGrid<PointG> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(size, size, size);
    vg.filter(*cloud);
}

Cylinder detection::ransac(PointCloudG cloud, float radius, float dis_threshold) {
    // Estimate point normals
    pcl::NormalEstimation<PointG, pcl::Normal> ne;
    pcl::search::KdTree<PointG>::Ptr tree (new pcl::search::KdTree<PointG> ());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (8);
    ne.compute (*cloud_normals);
    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<PointG, pcl::Normal> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (200000);
    seg.setDistanceThreshold (dis_threshold);
    seg.setRadiusLimits (radius*0.6, radius*1.4);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);
    // Obtain the cylinder inliers and coefficients
    pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    cout << "Ransac cylinder coefficients: " << *coefficients_cylinder << endl;
    // Write the cylinder inliers to disk
    pcl::ExtractIndices<PointG> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    extract.filter (*cloud);
    if (cloud->points.empty ())
        cerr << "Can't find the cylindrical component." << endl;
    else
        cout << "PointCloud representing the cylindrical component: " << cloud->points.size () << " data points." << endl;

    double temp[LEN_CYL];
    for (int i = 0; i < LEN_CYL; ++i)
        temp[(i+1)%LEN_CYL] = (double)coefficients_cylinder->values[i];
    return Cylinder(temp);
}

struct CYLINDER_FITTING_COST {
    CYLINDER_FITTING_COST(double x1, double x2, double x3, double r) : _x1(x1), _x2(x2), _x3(x3), _r(r){}
    template<typename T>
    bool operator()(
            const T *const theta,
            T *residual) const {
        residual[0] = ceres::sqrt((T(_x1)-theta[0])*(T(_x1)-theta[0])
                                  + (T(_x2)-theta[1])*(T(_x2)-theta[1])
                                  + (T(_x3)-theta[2])*(T(_x3)-theta[2])
                                  - ceres::pow((T(_x1)-theta[0])*theta[3]
                                  + (T(_x2)-theta[1])*theta[4]
                                  + (T(_x3)-theta[2])*theta[5], 2) / (theta[3]*theta[3]+theta[4]*theta[4]+theta[5]*theta[5])) - T(_r); // r-sqrt(...)
        return true;
    }
    const double _x1, _x2, _x3, _r;
};

Cylinder detection::fitting(PointCloudG cloud, Cylinder guess) {
    double theta[6] = {guess[1], guess[2], guess[3], guess[4], guess[5], guess[6]};
    double r = guess[0];
    int N = cloud->points.size ();
    vector<double> x1_data, x2_data, x3_data;
    for (int i = 0; i < N; i++) {
        x1_data.push_back(cloud->points[i].x);
        x2_data.push_back(cloud->points[i].y);
        x3_data.push_back(cloud->points[i].z);
    }
    ceres::Problem problem;
    for (int i = 0; i < N; i++) {
        problem.AddResidualBlock(
                new ceres::AutoDiffCostFunction<CYLINDER_FITTING_COST, 1, 6>(
                        new CYLINDER_FITTING_COST(x1_data[i], x2_data[i], x3_data[i], r)
                ),
                nullptr,
                theta
        );
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    // log
    cout << summary.BriefReport() << endl;
    // output cylinder
    double temp[LEN_CYL];
    temp[0] = r;
    for (int i = 0; i < LEN_CYL-1; ++i)
        temp[i+1] = theta[i];
    return Cylinder(temp);
}

Cylinder detection::finite_seg(PointCloudG cloud, Cylinder pole) {
    auto param_min = (double) INFINITY;
    auto param_max = (double) -INFINITY;
    Eigen::Vector3d start(pole[1], pole[2], pole[3]);
    Eigen::Vector3d dir(pole[4], pole[5], pole[6]);
    dir /= dir.norm();
    for (auto p : cloud->points) {
        double param = line_closest_param(start, dir, Eigen::Vector3d(p.x, p.y, p.z));
        if (param > param_max) param_max = param;
        if (param < param_min) param_min = param;
    }
    double length = param_max - param_min;
    double temp[LEN_CYL] = {pole[0], pole[1]+param_min*dir[0], pole[2]+param_min*dir[1], pole[3]+param_min*dir[2],
                            length*dir[0], length*dir[1], length*dir[2]};
    return Cylinder(temp);
}





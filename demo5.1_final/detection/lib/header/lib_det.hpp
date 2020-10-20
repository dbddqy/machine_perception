//
// Created by yue on 23.09.20.
//

#ifndef DETECTION_LIB_DET_HPP
#define DETECTION_LIB_DET_HPP

#include <lib_camera.hpp>
#include <utility.hpp>

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <opencv2/aruco.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#define LEN_T_4_4 16
#define LEN_TOPO_VEC 7

#define TOPO_TYPE 0
#define TOPO_FROM_INDEX 1
#define TOPO_FROM_M_PARAM 2
#define TOPO_FROM_B_PARAM 3
#define TOPO_TO_INDEX 4
#define TOPO_TO_M_PARAM 5
#define TOPO_TO_B_PARAM 6

typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

namespace detection {

    enum TASK_TYPE {
        SCAN_MATERIAL,
        SCAN_BUILT
    };

    enum TASK_NODE_TYPE {
        NODE_FROM,
        NODE_TO
    };

    PointCloudG solve_seg(cv::Mat depth, Eigen::Isometry3d c2w, Cylinder pole, double seg_param, double seg_length,
                          double dis_max=2.0, double dis_thresh=0.005);

    bool detectMarker(cv::Mat img, std::vector< std::vector<cv::Point2f> > &corners, std::vector<int> &ids, double size,
            bool draw = true, cv::aruco::CornerRefineMethod corner_refinement = cv::aruco::CORNER_REFINE_NONE);

    Eigen::Isometry3d solve_pose(std::vector <std::vector<cv::Point2f>> corners, std::vector<int> ids, double size,
            std::vector<int> indices, std::vector<Eigen::Matrix4d> marker_poses);

    void clean_cloud(PointCloudG cloud, int mean_k);

    void downsample_cloud(PointCloudG cloud, float size);

    void remove_plane(PointCloudG cloud);

    void pass_through(PointCloudG cloud, double pass_through_height);

    Cylinder ransac(PointCloudG cloud, float radius, float dis_threshold);

    Cylinder fitting(PointCloudG cloud, Cylinder guess);

    Cylinder finite_seg(PointCloudG cloud, Cylinder pole);

    void draw_axis(cv::Mat img, Cylinder pole, Eigen::Isometry3d c2w,
                  cv::Scalar color=cv::Scalar(255, 0, 0), int width=2);

    void draw_axes(cv::Mat img, std::vector<Cylinder> poles, Eigen::Isometry3d c2w,
                   cv::Scalar color=cv::Scalar(255, 0, 0), int width=2);

    double getDeviation(cv::Mat color, cv::Mat depth, Cylinder pole, Eigen::Isometry3d c2w, int divide,
                        cv::Scalar point_color=cv::Scalar(255, 0, 0), int size=2, int thickness=1, bool plot=true);
}

#endif //DETECTION_LIB_DET_HPP

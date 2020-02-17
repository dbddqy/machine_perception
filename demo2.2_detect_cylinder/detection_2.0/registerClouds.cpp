#include <iostream>
#include <boost/format.hpp>  // for formating strings

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

using namespace std;

pcl::PCDReader reader;
pcl::PCDWriter writer;
pcl::StatisticalOutlierRemoval<PointG> sor;

const int COUNT = 5;

int main(int argc, char **argv) {
    // c2o transformation
    double t[3] = {-40.3669, 60.2576, 167.664};
    double r[9] = {-0.073916, -0.997249, -0.00562505, 0.996009, -0.0735388, -0.050574, 0.0500212, -0.00934083, 0.998704};
    Eigen::Matrix4d c2o;
    c2o << r[0], r[3], r[6], t[0],
           r[1], r[4], r[7], t[1],
           r[2], r[5], r[8], t[2],
           0.0, 0.0, 0.0, 1.0;

    // cylinder params
    double cylinder[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    PointCloudG cloud_merged;
    for (int i = 0; i < COUNT; ++i) {
        PointCloudG cloud_c (new pcl::PointCloud<PointG>);
        boost::format fmt( "../data/cloud_full0%d.pcd" );
        reader.read ((fmt%i).str(), *cloud_c);
        PointCloudG cloud_w (new pcl::PointCloud<PointG>);
        pcl::transformPointCloud(*cloud_c, *cloud_w, c2o);

        vector<pcl::PointIndices> cluster_indices;

        for (std::vector<pcl::PointIndices>::const_iterator it = cloud_w->begin(); it != cloud_w->end(); it++)
        {
            // Return an Eigen::Vector3f of points coordinate.
            Eigen::Vector3d p = it->getVector3fMap().cast<double>();  // if double wanted, just say it->getVector3fMap().cast<double>
            double dis = (p[0]-cylinder[0]) * (p[0]-cylinder[0])
                    + (p[1]-cylinder[1]) * (p[1]-cylinder[1])
                    + (p[2]-cylinder[2]) * (p[2]-cylinder[2])
                    - (p[0]*cylinder[3] + p[1]*cylinder[4] + p[2]*cylinder[5]) * (p[0]*cylinder[3] + p[1]*cylinder[4] + p[2]*cylinder[5])
                    - cylinder[6] * cylinder[6];
            if (dis < 30)
                cloud_merged->push_back()
        }
    }
    // remove outliers
    sor.setInputCloud (cloud_merged);
    sor.setMeanK (50);
    sor.setStddevMulThresh (0.8);
    sor.filter (*cloud_merged);

    return 0;
}

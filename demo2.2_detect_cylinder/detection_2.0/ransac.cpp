//
// Created by yue on 18.02.20.
//

#include <string>
#include <boost/format.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

using namespace std;

pcl::PCDReader reader;
pcl::PCDWriter writer;

pcl::NormalEstimation<PointG, pcl::Normal> ne;
pcl::SACSegmentationFromNormals<PointG, pcl::Normal> seg;
pcl::search::KdTree<PointG>::Ptr tree (new pcl::search::KdTree<PointG> ());
pcl::ExtractIndices<PointG> extract;

pcl::PointCloud<PointG>::Ptr cloud (new pcl::PointCloud<PointG>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<PointG>::Ptr cloud_filtered (new pcl::PointCloud<PointG>);

pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);

void extractCylinder(string path) {
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.2);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0.01, 0.04);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointG>::Ptr cloud_cylinder (new pcl::PointCloud<PointG> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else {
        std::cout << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
        writer.write (path, *cloud_cylinder, false);
    }
}

int main(int argc, char **argv) {
    reader.read ("../test_data/data_1/cloud_merged.pcd", *cloud);
    std::cout << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    extractCylinder("../test_data/data_1/cloud_cylinder.pcd");
    return 0;
}
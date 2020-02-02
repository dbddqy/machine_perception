//
// Created by yue on 30.01.20.
//

#include <string>
#include <boost/format.hpp>

#include "pointCloud.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

pcl::PCDReader reader;
pcl::PCDWriter writer;

pcl::PassThrough<PointG> pass;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
pcl::NormalEstimation<PointG, pcl::Normal> ne;
pcl::SACSegmentationFromNormals<PointG, pcl::Normal> seg;
pcl::search::KdTree<PointG>::Ptr tree (new pcl::search::KdTree<PointG> ());
pcl::ExtractIndices<PointG> extract;
pcl::ExtractIndices<pcl::Normal> extract_normals;

pcl::PointCloud<PointG>::Ptr cloud (new pcl::PointCloud<PointG>);
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<PointG>::Ptr cloud_filtered (new pcl::PointCloud<PointG>);

pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
pcl::PointIndices::Ptr inliers_sor (new pcl::PointIndices);

void extractCylinder(string path) {
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.2);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.1);
    seg.setRadiusLimits (0.01, 0.04);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // test
//    cout << coefficients_cylinder->values.at(0) << endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered);
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

    // get rid of the detected cylinder
    extract.setNegative (true);
    extract.filter (*cloud_filtered);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_cylinder);
    extract_normals.filter (*cloud_normals);

    // get rid of the outliers
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

//    sor.getRemovedIndices(*inliers_sor);
//    extract_normals.setInputCloud (cloud_normals);
//    extract_normals.setIndices (inliers_sor);
//    extract_normals.filter (*cloud_normals);
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);
}

int main() {
    reader.read ("../../data/point_cloud/cloud1_alignedB.pcd", *cloud);
    std::cout << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 5.0);
    pass.filter (*cloud_filtered);

    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);

    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;
//    writer.write ("../../data/point_cloud/cloud_f2.pcd", *cloud_filtered, false);

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    for (int i = 0; i < 2; ++i) {
        boost::format fmt( "../../data/point_cloud/cloud_cylinder_detected0%d.pcd" );
        extractCylinder((fmt%i).str());
    }

    std::cout << "PointCloud has: " << cloud_filtered->points.size () << " data points left." << std::endl;
    writer.write ("../../data/point_cloud/cloud_cylinder_detected_rest.pcd", *cloud_filtered, false);
}

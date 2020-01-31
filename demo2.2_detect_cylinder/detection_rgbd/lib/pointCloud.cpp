//
// Created by yue on 26.01.20.
//

#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>

#include <librealsense2/rs.hpp>

#include "pointCloud.hpp"

PointCloudG points2pcl(const rs2::points& points) {
    PointCloudG cloud(new pcl::PointCloud<pcl::PointXYZ>);
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points) {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
    return cloud;
}

void detectCylinderRANSAC(const PointCloudG& pc) {
    // All the objects needed
//    pcl::PCDReader reader;
    pcl::PassThrough<PointG> pass;
    pcl::NormalEstimation<PointG, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointG, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<PointG> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<PointG>::Ptr tree (new pcl::search::KdTree<PointG> ());

    // Datasets
    pcl::PointCloud<PointG>::Ptr cloud (new pcl::PointCloud<PointG>);
    pcl::PointCloud<PointG>::Ptr cloud_filtered (new pcl::PointCloud<PointG>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointG>::Ptr cloud_filtered2 (new pcl::PointCloud<PointG>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

    // Read in the cloud data
//    reader.read ("cloud_0.pcd", *cloud);
    cloud = pc;
    std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

    // Build a passthrough filter to remove spurious NaNs
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.2, 2.0);
    pass.filter (*cloud_filtered);
    std::cerr << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << std::endl;

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);
    std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointG>::Ptr cloud_plane (new pcl::PointCloud<PointG> ());
    extract.filter (*cloud_plane);
    std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    writer.write ("plane.pcd", *cloud_plane, false);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_filtered2);
    extract_normals.setNegative (true);
    extract_normals.setInputCloud (cloud_normals);
    extract_normals.setIndices (inliers_plane);
    extract_normals.filter (*cloud_normals2);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (0.1);
    seg.setMaxIterations (10000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits (0, 0.1);
    seg.setInputCloud (cloud_filtered2);
    seg.setInputNormals (cloud_normals2);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

    // Write the cylinder inliers to disk
    extract.setInputCloud (cloud_filtered2);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointG>::Ptr cloud_cylinder (new pcl::PointCloud<PointG> ());
    extract.filter (*cloud_cylinder);
    if (cloud_cylinder->points.empty ())
        std::cerr << "Can't find the cylindrical component." << std::endl;
    else
    {
        std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points." << std::endl;
        writer.write ("cylinder.pcd", *cloud_cylinder, false);
    }
}

int test() {
    pcl::visualization::CloudViewer viewer("viewer");

    pcl::PCDReader reader;
    pcl::PassThrough<PointG> pass;
    pcl::NormalEstimation<PointG, pcl::Normal> ne;
    pcl::search::KdTree<PointG>::Ptr tree(new pcl::search::KdTree<PointG> ());

    pcl::PointCloud<PointG>::Ptr cloud(new pcl::PointCloud<PointG>);
    pcl::PointCloud<PointG>::Ptr cloud_filtered (new pcl::PointCloud<PointG>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    reader.read ("../../data/point_cloud/cloud_0.pcd", *cloud);
    cout << "PointCloud has: " << cloud->points.size () << " data points." << endl;
    cout << "width: " << cloud->width << endl;
    cout << "height: " << cloud->height << endl;

    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 0.8);
    pass.filter(*cloud_filtered);
    cout << "PointCloud after filtering has: " << cloud_filtered->points.size () << " data points." << endl;

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    viewer.showCloud(cloud_filtered);
    while (!viewer.wasStopped())
    {
    }

    return 0;
}

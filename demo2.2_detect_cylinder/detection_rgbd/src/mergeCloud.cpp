//
// Created by yue on 01.02.20.
//

#include <string>
#include <boost/format.hpp>

#include "pointCloud.hpp"

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>

pcl::PCDReader reader;
pcl::PCDWriter writer;
pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

const int COUNT = 5;

int main() {
    PointCloudG clouds[COUNT];
    for (int i = 0; i < COUNT; ++i) {
        PointCloudG cloud (new pcl::PointCloud<PointG>);
        boost::format fmt( "../../data/point_cloud/cloud1_%d%s.pcd" );
        reader.read ((fmt%i%"B").str(), *cloud);
        // remove outliers
        sor.setInputCloud (cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (0.8);
        sor.filter (*cloud);
        clouds[i] = cloud;
    }

    PointCloudG cloud_aligned = clouds[0];
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    cout << "cloud 0: " << clouds[0]->points.size() << endl;
    for (int i = 1; i < COUNT; ++i) {
        cout << "cloud " << i << ": " << clouds[i]->points.size() << endl;
        icp.setInputSource(clouds[i]);
        icp.setInputTarget(cloud_aligned);
        pcl::PointCloud<pcl::PointXYZ> final;
        icp.align(final);
        *cloud_aligned += final;
    }

    sor.setInputCloud (cloud_aligned);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_aligned);
    cout << "final: " << cloud_aligned->points.size() << endl;

    writer.write ("../../data/point_cloud/cloud_aligned.pcd", *cloud_aligned, false);
}
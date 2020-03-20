//
// Created by yue on 18.02.20.
//

#include <iostream>
#include <boost/format.hpp>  // for formating strings

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

using namespace std;

pcl::PCDReader reader;
pcl::PCDWriter writer;
pcl::StatisticalOutlierRemoval<PointG> sor;

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "please enter the file path." << endl;
        return -1;
    }
    PointCloudG cloud (new pcl::PointCloud<PointG>);
    reader.read (argv[1], *cloud);
    sor.setInputCloud (cloud);
    sor.setMeanK (10);
    sor.setStddevMulThresh (0.5);
    sor.filter (*cloud);
    writer.write ("cloud_cleaned.pcd", *cloud, false);
    return 0;
}
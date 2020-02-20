//
// Created by ruqing on 20.02.20.
//

#include <iostream>
#include <fstream>
#include <boost/format.hpp>  // for formating strings

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

using namespace std;

pcl::PCDReader reader;
pcl::PCDWriter writer;

int main(int argc, char **argv) {
    if (argc != 3) {
        cout << "please enter the file path and the voxel size." << endl;
        return -1;
    }
    PointCloudG cloud (new pcl::PointCloud<PointG>);
    PointCloudG cloud_filtered (new pcl::PointCloud<PointG>);
    reader.read (argv[1], *cloud);

    pcl::VoxelGrid<PointG> sor;
    sor.setInputCloud(cloud);
    double size = stod(argv[2]);
    sor.setLeafSize(size, size, size);
    sor.filter(*cloud_filtered);

    writer.write ("cloud_downsampled.pcd", *cloud_filtered, false);
    return 0;
}
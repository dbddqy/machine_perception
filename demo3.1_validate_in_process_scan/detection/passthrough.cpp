//
// Created by yue on 19.03.20.
//

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

using namespace std;

pcl::PCDReader reader;
pcl::PCDWriter writer;
pcl::PassThrough<pcl::PointXYZ> pass;

int main(int argc, char **argv) {
    PointCloudG cloud(new pcl::PointCloud<PointG>);
    reader.read("../data3D/cloud_aligned.pcd", *cloud);

    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (75.1, 333.6);
    pass.filter (*cloud);
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-82.4, 8.6);
    pass.filter (*cloud);
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (34.25, 900);
    pass.filter (*cloud);

    writer.write ("../data3D/cloud_passthrough.pcd", *cloud, false);
}
//
// Created by ruqing on 20.02.20.
//

#include <iostream>
#include <fstream>
#include <chrono>
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
//    if (argc != 3) {
//        cout << "please enter the file path and the voxel size." << endl;
//        return -1;
//    }

//    double time_total = 0.0;
    int num_frames = 1;
    for (int frame_index = 0; frame_index < num_frames; ++frame_index) {
        PointCloudC cloud(new pcl::PointCloud<PointC>);
        PointCloudC cloud_filtered(new pcl::PointCloud<PointC>);
        boost::format fmt("../data3D/cloud_full_%d.pcd");
        reader.read((fmt % frame_index).str(), *cloud);

        pcl::VoxelGrid<PointC> sor;
        sor.setInputCloud(cloud);
        float size = 4.0f;
        sor.setLeafSize(size, size, size);

//        chrono::steady_clock::time_point t_start = chrono::steady_clock::now(); // timing start

        sor.filter(*cloud_filtered);

//        chrono::steady_clock::time_point t_end = chrono::steady_clock::now(); // timing end
//        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t_end - t_start);
//        cout << "time cost = " << time_used.count() << " seconds. " << endl;
//        time_total += time_used.count();

        boost::format fmt_out( "../data3D/cloud_downsampled_%d.pcd" );
        writer.write((fmt_out % frame_index).str(), *cloud_filtered, false);
    }
//    cout << "time total cost = " << time_total << " seconds. " << endl;
    return 0;
}
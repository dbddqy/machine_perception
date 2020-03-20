#include <iostream>
#include <fstream>
#include <chrono>
#include <boost/format.hpp>  // for formating strings

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

using namespace std;

pcl::PCDReader reader;
pcl::PCDWriter writer;
pcl::StatisticalOutlierRemoval<PointG> sor;

int main(int argc, char **argv) {
    int num_frames = 4;

    PointCloudG clouds_full[num_frames];
    PointCloudG cloud_aligned (new pcl::PointCloud<PointG>);
    // load all the clouds
    for (int frame_index = 0; frame_index < num_frames; ++frame_index) {
        PointCloudG cloud(new pcl::PointCloud<PointG>);
        boost::format fmt("../data3D/cloud_downsampled_%d.pcd");
        reader.read((fmt % frame_index).str(), *cloud);
        clouds_full[frame_index] = cloud;
    }
    cloud_aligned = clouds_full[0];
    cout << "cloud 0: " << clouds_full[0]->points.size() << endl;
    for (int frame_index = 1; frame_index < num_frames; ++frame_index) {
        cout << "cloud " << frame_index << ": " << clouds_full[frame_index]->points.size() << endl;
        *cloud_aligned += *clouds_full[frame_index];
    }

    // remove outliers
//    sor.setInputCloud (cloud_merged);
//    sor.setMeanK (5);
//    sor.setStddevMulThresh (0.5);
//    sor.filter (*cloud_merged);

//    cloud_merged->width = cloud_merged->points.size();
//    cloud_merged->height = 1;
//    cloud_merged->resize(cloud_merged->width*cloud_merged->height);
    writer.write ("../data3D/cloud_aligned.pcd", *cloud_aligned, false);
    return 0;
}

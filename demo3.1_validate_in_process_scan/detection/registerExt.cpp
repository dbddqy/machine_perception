#include <iostream>
#include <fstream>
#include <chrono>
#include <boost/format.hpp>  // for formating strings

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZRGB P_C;
typedef pcl::PointXYZ P_G;
typedef pcl::PointCloud<P_C>::Ptr PC_C;
typedef pcl::PointCloud<P_G>::Ptr PC_G;

using namespace std;
using namespace cv;

pcl::PCDReader reader;
pcl::PCDWriter writer;
pcl::StatisticalOutlierRemoval<P_C> sor;
pcl::VoxelGrid<P_C> vg;

class CameraConfig {
public:
    Mat M;
    Mat distortion;
    Mat MInv;
    double fx, fy, cx, cy;

    CameraConfig() {
        M = (Mat_<double>(3, 3)
                << 1382.23, 0., 953.567
                , 0., 1379.46, 532.635
                , 0., 0., 1.);
        distortion = (Mat_<double >(1, 5)
                << 0.0, 0.0, 0.0, 0.0, 0.0);
        MInv = M.inv();
        fx = 1382.23; fy = 1379.46; cx = 953.567; cy = 532.635;
    };
};

CameraConfig C;

double **readData(string path, int num_frames) {
    ifstream file;
    file.open(path, ios::in);
    auto **data = new double*[num_frames];
    for (int i = 0; i < num_frames; ++i) {
        data[i] = new double[6];
        for (int j = 0; j < 6; ++j)
            file >> data[i][j];
    }
    file.close();
    return data;
}

Mat t_4_4__rvec(Vec3d rvec, Vec3d tvec) {
    Mat r, t_4_4;
    Rodrigues(rvec, r);
    hconcat(r, tvec*1000.0, t_4_4);
    Mat down = (Mat_<double>(1, 4) << 0., 0., 0., 1.);
    vconcat(t_4_4, down, t_4_4);
    return t_4_4;
}

int findIndex(vector<int> indices, int index) {
    auto iElement = find(indices.begin(), indices.end(), index);
    if( iElement != indices.end() ) {
        return distance(indices.begin(),iElement);
    } else {
        return -1;
    }
}

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "please enter number of frames" << endl;
        return -1;
    }
    const int num_frames = stoi(argv[1]);

    // read ext params
    double **ext_params = readData("../data2D/c2w.txt", num_frames);
    vector<Mat> w2c;
    for (int i = 0; i < num_frames; ++i) {
        Vec3d rvec(ext_params[i][0], ext_params[i][1], ext_params[i][2]);
        Vec3d tvec(ext_params[i][3], ext_params[i][4], ext_params[i][5]);
        w2c.push_back(t_4_4__rvec(rvec, tvec).inv());
//        cout << c2w[i] << endl;
//        for (int j = 0; j < 6; ++j) {
//            cout << ext_params[i][j] << " ";
//        }
//        cout << endl;
    }

//    PointCloudC clouds_full[num_frames];
    PC_C cloud_aligned (new pcl::PointCloud<P_C>);
    vector<int> skip_indices = {40, 41, 42, 43, 44, 46,
                                49,
                                52};
    // load clouds
    for (int frame_index = 0; frame_index < num_frames; ++frame_index) {
        // skip index
        if (findIndex(skip_indices, frame_index) != -1) continue;
        // load image
        Mat matColor = imread((boost::format("../data2D/color_%d.png") % frame_index).str());
        Mat matDepth = imread((boost::format("../data2D/depth_%d.png") % frame_index).str(), CV_LOAD_IMAGE_UNCHANGED);
        // construct cloud
        PC_C cloud_t (new pcl::PointCloud<P_C>);
        for (int v = 0; v < matColor.rows; ++v) {
            for (int u = 0; u < matColor.cols; ++u) {
                double d = (double)matDepth.ptr<uint16_t>(v)[u] * 0.1; // unit: 0.1mm
                if (d == 0.0 || d > 1500.0f) continue;
                Mat p_c = (Mat_<double>(4, 1)
                        << ((double)u - C.cx) * d / C.fx, ((double)v - C.cy) * d / C.fy, d, 1.0);
                Mat p_w = w2c[frame_index] * p_c;
                P_C pt;
                pt.x = p_w.at<double>(0, 0);
                pt.y = p_w.at<double>(1, 0);
                pt.z = p_w.at<double>(2, 0);
                pt.b = matColor.data[ v*matColor.step+u*matColor.channels() ];
                pt.g = matColor.data[ v*matColor.step+u*matColor.channels()+1 ];
                pt.r = matColor.data[ v*matColor.step+u*matColor.channels()+2 ];
                cloud_t->points.push_back(pt);
            }
        }

        vg.setInputCloud(cloud_t);
        float size = 2.0f;
        vg.setLeafSize(size, size, size);
        vg.filter(*cloud_t);
        // remove outliers
        sor.setInputCloud (cloud_t);
        sor.setMeanK (20);
        sor.setStddevMulThresh (0.5);
        sor.filter (*cloud_t);

        *cloud_aligned += *cloud_t;
    }

//    cloud_aligned = clouds_full[0];
//    pcl::IterativeClosestPoint<PointG, PointG> icp;
//    cout << "cloud 0: " << clouds_full[0]->points.size() << endl;
//    for (int frame_index = 1; frame_index < num_frames; ++frame_index) {
//        cout << "cloud " << frame_index << ": " << clouds_full[frame_index]->points.size() << endl;
//
////        chrono::steady_clock::time_point t_start = chrono::steady_clock::now(); // timing start
//
//        icp.setInputSource(clouds_full[frame_index]);
//        icp.setInputTarget(cloud_aligned);
//        pcl::PointCloud<PointG> final;
//        icp.align(final);
//        *cloud_aligned += final;
//
////        chrono::steady_clock::time_point t_end = chrono::steady_clock::now(); // timing end
////        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t_end - t_start);
////        cout << "time cost = " << time_used.count() << " seconds. " << endl;
//    }
//

//
////    cloud_merged->width = cloud_merged->points.size();
////    cloud_merged->height = 1;
////    cloud_merged->resize(cloud_merged->width*cloud_merged->height);
    vg.setInputCloud(cloud_aligned);
    float size = 4.0f;
    vg.setLeafSize(size, size, size);
    vg.filter(*cloud_aligned);
    writer.write ("../data3D/cloud_aligned.pcd", *cloud_aligned, false);
    return 0;
}
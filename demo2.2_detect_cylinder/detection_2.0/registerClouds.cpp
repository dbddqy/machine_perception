#include <iostream>
#include <fstream>
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

const int COUNT = 11;

int main(int argc, char **argv) {
    // e2c transformation
    double t[3] = {-0.0153632, 0.042317, 0.139266};
//    double r[9] = {-0.0, -1.0, -0.0, 1.0, -0.0, 0.00, -0.00, -0.0, 1.0};
    double r[9] = {-0.000174572, -1, -0.000177886, 0.999979, -0.000175714, 0.00644069, -0.00644072, -0.000176758, 0.999979};
    Eigen::Matrix4d e2c;
    e2c << r[0], r[3], r[6], t[0],
           r[1], r[4], r[7], t[1],
           r[2], r[5], r[8], t[2],
           0.0, 0.0, 0.0, 1.0;
    // w2e transformation
    ifstream file;
    file.open("../test_data/data2/robot_pose.txt", ios::in); // KUKA X Y Z A B C (in mm, degree)
    vector<Eigen::Matrix4d> w2e;
    for (int i = 0; i < COUNT; ++i) {
        double data[6];
        for (auto& d : data)
            file >> d;
        Eigen::Quaterniond q = Eigen::AngleAxisd(data[3]*M_PI/180.0, Eigen::Vector3d::UnitZ())
                            * Eigen::AngleAxisd(data[4]*M_PI/180.0, Eigen::Vector3d::UnitY())
                            * Eigen::AngleAxisd(data[5]*M_PI/180.0, Eigen::Vector3d::UnitX());
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0]/1000.0, data[1]/1000.0, data[2]/1000.0));
        w2e.push_back(T.matrix());
    }
    file.close();
    // cylinder params
//    double cylinder[7] = {1.62165, -0.09601, 0.17847, 0.0, 1.0, 0.0, 0.0158};
    double cylinder[7] = {0.05423, -2.47415, 1.32664, 0.0, 1.0, 0.0, 0.0158};  // 2cut
//    double cylinder[7] = {1.68594, 0.5306, 0.24832, 0.55457148246, 0.83183737076, -0.02229482977, 0.018685};  //No.2
//    double cylinder[7] = {1.61857, 0.7942, 0.29441, 0.91515029888, -0.26184958961, -0.30648772059, 0.019085};  //No.3

    PointCloudG clouds[COUNT];
    PointCloudG cloud_merged (new pcl::PointCloud<PointG>);
    for (int i = 0; i < COUNT; ++i) {
        PointCloudG cloud_c (new pcl::PointCloud<PointG>);
        boost::format fmt( "../test_data/data2/cloud_full0%d.pcd" );
        reader.read ((fmt%i).str(), *cloud_c);
//        PointCloudG cloud_e (new pcl::PointCloud<PointG>);
//        pcl::transformPointCloud(*cloud_c, *cloud_e, e2c.inverse().cast<float>());
//        PointCloudG cloud_w (new pcl::PointCloud<PointG>);
//        pcl::transformPointCloud(*cloud_e, *cloud_w, w2e[i].inverse().cast<float>());

        double min_dis = 999999999.0;
        double max_dis = 0.0;
        PointCloudG cloud_cylinder (new pcl::PointCloud<PointG>);
        for (int j=0; j < cloud_c->points.size(); j++) {
//            PointG p = cloud_w->points[j];
//            double dis = sqrt((p.x-cylinder[0]) * (p.x-cylinder[0])
//                    + (p.y-cylinder[1]) * (p.y-cylinder[1])
//                    + (p.z-cylinder[2]) * (p.z-cylinder[2])
//                    - pow((p.x-cylinder[0])*cylinder[3] + (p.y-cylinder[1])*cylinder[4] + (p.z-cylinder[2])*cylinder[5], 2))
//                    - cylinder[6];
            Eigen::Vector4d p_c;
            p_c << cloud_c->points[j].x, cloud_c->points[j].y, cloud_c->points[j].z, 1.0;
            Eigen::Vector4d p = w2e[i] * e2c * p_c;
            double dis = sqrt((p[0]-cylinder[0]) * (p[0]-cylinder[0])
                    + (p[1]-cylinder[1]) * (p[1]-cylinder[1])
                    + (p[2]-cylinder[2]) * (p[2]-cylinder[2])
                    - pow((p[0]-cylinder[0])*cylinder[3] + (p[1]-cylinder[1])*cylinder[4] + (p[2]-cylinder[2])*cylinder[5], 2))
                    - cylinder[6];
//            if (j%1000 == 0) {
//                cout << "j: " << j << endl;
//                cout << "x: " << p.x << " y: " << p.y << " z: " << p.z << endl;
//                cout << dis << endl;
//            }
            if (dis < min_dis)
                min_dis = dis;
            if (dis > max_dis)
                max_dis = dis;
            if (abs(dis) < 0.01) {
//                cloud_cylinder->points.push_back(cloud_c->points[j]);
                PointG p2append;
                p2append.x = p[0]; p2append.y = p[1]; p2append.z = p[2];
                cloud_merged->points.push_back(p2append);
            }
        }
//        clouds[i] = cloud_cylinder;
        cout << "min: " << min_dis << endl;
        cout << "max: " << max_dis << endl;
    }

//    cloud_merged = clouds[0];
//    pcl::IterativeClosestPoint<PointG, PointG> icp;
//    cout << "cloud 0: " << clouds[0]->points.size() << endl;
//    for (int i = 1; i < COUNT; ++i) {
//        cout << "cloud " << i << ": " << clouds[i]->points.size() << endl;
//        icp.setInputSource(clouds[i]);
//        icp.setInputTarget(cloud_merged);
//        pcl::PointCloud<PointG> final;
//        icp.align(final);
//        *cloud_merged += final;
//    }
//    cout << icp.getFinalTransformation() << endl;

    // remove outliers
//    sor.setInputCloud (cloud_merged);
//    sor.setMeanK (5);
//    sor.setStddevMulThresh (0.5);
//    sor.filter (*cloud_merged);

    cloud_merged->width = cloud_merged->points.size();
    cloud_merged->height = 1;
    cloud_merged->resize(cloud_merged->width*cloud_merged->height);
    writer.write ("../test_data/data2/cloud_merged.pcd", *cloud_merged, false);
    return 0;
}

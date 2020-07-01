//
// Created by yue on 29.03.20.
//

#include "construct3D_built.hpp"

#include <iostream>
#include <chrono>
#include <string>
#include <boost/format.hpp>  // for formating strings

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointXYZ PointG;
typedef pcl::PointCloud<PointC>::Ptr PointCloudC;
typedef pcl::PointCloud<PointG>::Ptr PointCloudG;

using namespace std;
using namespace cv;

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

bool detectCircleboard(Mat img, Mat &tranc2o, Point &center) {
    vector<Point2f> centers;
    bool pattern_was_found = findCirclesGrid(img, cv::Size(2, 13), centers, CALIB_CB_ASYMMETRIC_GRID);
    if (!pattern_was_found)
        return false;
    cv::drawChessboardCorners(img, cv::Size(2, 13), Mat(centers), pattern_was_found);
    vector<Point3f> objectPoints;

    for (int i = 0; i < 13; ++i)
        for (int j = 0; j < 2; ++j)
            objectPoints.push_back(Point3d(i*0.02, j*0.04+(i%2)*0.02, 0.0));
    Mat rvec, tvec;
    Mat matCorneres = Mat(centers).reshape(1);
    Mat matObjectPoints = Mat(objectPoints).reshape(1);
    solvePnP(matObjectPoints, matCorneres, C.M, C.distortion, rvec, tvec);

    cv::aruco::drawAxis(img, C.M, C.distortion, rvec, tvec, 0.04);
    center = centers[0];
    Mat r;
    Rodrigues(rvec, r);
    hconcat(r, tvec*1000.0, tranc2o);
    Mat down = (Mat_<double>(1, 4) << 0., 0., 0., 1.);
    vconcat(tranc2o, down, tranc2o);
    return true;
}

double **readData(string path) {
    ifstream file;
    file.open(path, ios::in);
    double **data = new double*[7];
//    double data[POLE_COUNT][7] = {0}; // every cylinder has 7 params
    for (int i = 0; i < POLE_COUNT; ++i) {
        data[i] = new double[7];
        for (int j = 0; j < 7; ++j)
            file >> data[i][j];
    }
    file.close();
    return data;
}

double dis(Vec3d p1, Vec3d p2) {
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1])+(p1[2]-p2[2])*(p1[2]-p2[2]));
}

void grow(cv::Mat& src, cv::Mat& mask, cv::Point seed, int threshold) {
    /* apply "seed grow" in a given seed
     * Params:
     *   src: source image
     *   mask: a matrix records the region found in current "seed grow"
     */
    const cv::Point PointShift2D[4] =
            {
                    cv::Point(1, 0),
                    cv::Point(0, -1),
                    cv::Point(-1, 0),
                    cv::Point(0, 1),
            };

    stack<cv::Point> point_stack;
    point_stack.push(seed);

    while(!point_stack.empty()) {
        cv::Point center = point_stack.top();
        mask.at<uchar>(center) = 1;
        point_stack.pop();

        for (int i=0; i<4; ++i) {
            cv::Point estimating_point = center + PointShift2D[i];
            if (estimating_point.x < 0
                || estimating_point.x > src.cols-1
                || estimating_point.y < 0
                || estimating_point.y > src.rows-1) {
                // estimating_point should not out of the range in image
                continue;
            } else {
                int delta=0;
                if (src.type() == CV_16U)
                    delta = (int)abs(src.at<uint16_t>(center) - src.at<uint16_t>(estimating_point));
                    // delta = (R-R')^2 + (G-G')^2 + (B-B')^2
                else
                    delta = int(pow(src.at<cv::Vec3b>(center)[0] - src.at<cv::Vec3b>(estimating_point)[0], 2)
                                + pow(src.at<cv::Vec3b>(center)[1] - src.at<cv::Vec3b>(estimating_point)[1], 2)
                                + pow(src.at<cv::Vec3b>(center)[2] - src.at<cv::Vec3b>(estimating_point)[2], 2));
                if (mask.at<uchar>(estimating_point) == 0
                    && delta < threshold) {
                    mask.at<uchar>(estimating_point) = 1;
                    point_stack.push(estimating_point);
                }
            }
        }
    }
}

double distance2cylinder(double *cylinder_param, PointG pt, double start_param, double end_param) {
    Vec3d start(cylinder_param[1], cylinder_param[2], cylinder_param[3]);
    Vec3d dir(cylinder_param[4], cylinder_param[5], cylinder_param[6]);
    Vec3d p_start = start + start_param * dir;
    Vec3d p_end = start + end_param * dir;
    Vec3d p(pt.x, pt.y, pt.z);
    if ((p-p_start).dot(dir) < 0) return 0; // 0: not in the segment
    if ((p_end-p).dot(dir) < 0) return 0; // 0: not in the segment
    return sqrt((p-p_start).dot(p-p_start)-pow((p-p_start).dot(dir), 2)) - cylinder_param[0];
}

pcl::PCDWriter writer;

int main(int argc, char **argv) {
    if (argc != 5) {
        cout << "please enter number of frames, pole index and the start_param and end_param of the segment." << endl;
        return -1;
    }
    const int NUM_FRAMES = stoi(argv[1]);
    const int POLE_INDEX = stoi(argv[2]);
    const double SRART_PARAM = stod(argv[3]);
    const double END_PARAM = stod(argv[4]);

    // read color frame
    Mat matColor = imread("../data2D/color.png");
    // get marker pose
    Mat tran_c2o(Size(4, 4), CV_64F);
    Point center;
    if (detectCircleboard(matColor, tran_c2o, center)) {
        cout << "marker pose : "<< endl;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                cout << tran_c2o.at<double>(i, j) << " ";
        cout << endl;
    }
    Mat tran_o2c = tran_c2o.inv();

    // read depth frames
    Mat matsDepth[NUM_FRAMES];
    for (int frame_index = 0; frame_index < NUM_FRAMES; ++frame_index)
        matsDepth[frame_index] = imread((boost::format("../data2D/depth_%d.png") % frame_index).str(), CV_LOAD_IMAGE_UNCHANGED);

    // read cylinder params
    double **cylinder_params = readData("../construct3D_built/cylinder_params.txt");

    Mat mask = Mat::zeros(matColor.size(), CV_8UC1);
    Mat mask_blue = Mat::zeros(matColor.size(), CV_8UC3);
    PointCloudG cloud_full(new pcl::PointCloud<PointG>);
    for (int v = 0; v < matColor.rows; ++v) {
        for (int u = 0; u < matColor.cols; ++u) {
            // get the average depth value from all the frames of images
            double dis_average = 0.0;
            bool use_frame = true;
            for  (int frame_index = 0; frame_index < NUM_FRAMES; ++frame_index) {
                double d = (double)matsDepth[frame_index].ptr<uint16_t>(v)[u] * 0.1; // unit: 0.1mm
                if (d == 0.0) { use_frame = false; break;}
                dis_average += d;
            }
            if (!use_frame) continue;
            dis_average /= NUM_FRAMES;
            // 3d reconstruction
            if (dis_average == 0.0 || dis_average > 1500.0f) continue;
            Mat p_c = (Mat_<double>(4, 1)
                    << ((double)u - C.cx) * dis_average / C.fx, ((double)v - C.cy) * dis_average / C.fy, dis_average, 1.0);
            Mat p_o = tran_o2c * p_c;
            PointG pt(p_o.at<double>(0, 0), p_o.at<double>(1, 0), p_o.at<double>(2, 0));
//            double dis2cylinder = distance2cylinder(cylinder_params[POLE_INDEX], pt, SRART_PARAM, END_PARAM);
//            if (dis2cylinder != 0 && abs(dis2cylinder) < 5.0 && pt.z > -15.0)
//            if (pt.x < -60.0 && pt.y < 30.0 && pt.z > 390.0) // built
            if ( (pt.x < 0.0 && pt.y < 0.0 && pt.z > 360.5 && pt.z < 380.5)
                || (pt.x > 0.0 && pt.y < 0.0 && pt.z > 350.5 && pt.z < 358.5)
                   || (pt.x > 0.0 && pt.y > 0.0 && pt.z > 370.5 && pt.z < 386.5)
                      || (pt.x < 0.0 && pt.y > 0.0 && pt.z > 374.5 && pt.z < 382.5) ) // built 2
            {
                mask.at<uchar>(v, u) = 255;
                mask_blue.at<Vec3b>(v, u) = Vec3b(255, 0, 0);
                cloud_full->points.push_back(pt);
            } else {
                mask_blue.at<Vec3b>(v, u) = Vec3b(0, 0, 0);
            }
        }
    }
    Mat colorWithMask;
    matColor.copyTo(colorWithMask, mask);

//    // detect holes
    vector<vector<Point> > contours;
    findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    drawContours(matColor, contours, -1, Scalar(255, 0, 0));
//    cout << contours.size() << endl;
//    Rect2d rect = boundingRect(contours[0]);
//    Mat colorWithMaskRect =  matColor(rect);
//    cvtColor(colorWithMaskRect, colorWithMaskRect, CV_BGR2GRAY);
//    vector<Vec3f> circles;
//    HoughCircles(colorWithMaskRect, circles, CV_HOUGH_GRADIENT, 1.2, 10, 100, 15, 6, 12);
//    vector<PointCloudG> clouds_hole;
//    for (auto c : circles) {
//        cout << "hole found!" << endl;
//        circle(matColor, Point(c[0]+rect.x, c[1]+rect.y), c[2]*2, Scalar(0, 0, 255));
//        // add points on the circle to the clouds
//        vector<Point> points_on_circle;
//        ellipse2Poly(Point(c[0]+rect.x, c[1]+rect.y), Size(c[2]-2, c[2]-2), 0, 0, 360, 1, points_on_circle);
//        PointCloudG cloud_hole(new pcl::PointCloud<PointG>);
//        for (auto p : points_on_circle) {
//            double dis_average = 0.0;
//            bool use_frame = true;
//            for  (int frame_index = 0; frame_index < NUM_FRAMES; ++frame_index) {
//                double d = (double)matsDepth[frame_index].ptr<uint16_t>(p.y)[p.x] * 0.1; // unit: 0.1mm
//                if (d == 0.0) { use_frame = false; break;}
//                dis_average += d;
//            }
//            if (!use_frame) continue;
//            dis_average /= NUM_FRAMES;
//            // 3d reconstruction
//            if (dis_average == 0.0 || dis_average > 1500.0f) continue;
//            Mat p_c = (Mat_<double>(4, 1)
//                    << ((double)p.x - C.cx) * dis_average / C.fx, ((double)p.y - C.cy) * dis_average / C.fy, dis_average, 1.0);
//            Mat p_o = tran_o2c * p_c;
//            PointG pt(p_o.at<double>(0, 0), p_o.at<double>(1, 0), p_o.at<double>(2, 0));
//            cloud_hole->points.push_back(pt);
//        }
//        clouds_hole.push_back(cloud_hole);
//    }

    addWeighted(matColor, 1.0, mask_blue, 0.4, 0.0, matColor);
    imshow("chessboard", matColor);
    imwrite("../data2D/color_detected.png", matColor);
//    imshow("colorWithMaskHoles", colorWithMaskHoles);
    waitKey();

//        chrono::steady_clock::time_point t_start = chrono::steady_clock::now(); // timing start

//        chrono::steady_clock::time_point t_end = chrono::steady_clock::now(); // timing end
//        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t_end - t_start);
//        cout << "time cost = " << time_used.count() << " seconds. " << endl;
    cloud_full->width = cloud_full->points.size();
    cloud_full->height = 1;
    writer.write("../data3D/cloud_passthrough.pcd", *cloud_full, false);
    cout << "PointCloud has: " << cloud_full->points.size() << " data points." << endl;

//    // save clouds_hole
//    for (int i = 0; i < clouds_hole.size(); ++i) {
//        clouds_hole[i]->width = clouds_hole[i]->points.size();
//        clouds_hole[i]->height = 1;
//        writer.write((boost::format("../data3D/circle_%d.pcd")%i).str(), *(clouds_hole[i]), false);
//    }

    return 0;
}
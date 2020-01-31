//
// Created by yue on 03.12.19.
//

#include <detection.hpp>

vector<Point> findLargestContour(vector< vector<Point> > contours) {
    double largestArea = 0.; int largestIndex = 0;
    for (int i = 0; i < contours.size(); ++i) {
        double area = contourArea(contours[i]);
        if (area > largestArea){
            largestArea = area;
            largestIndex = i;
        }
    }
    return contours[largestIndex];
}

vector< vector<Point> > findLargeContours(vector< vector<Point> > contours, double minArea) {
    vector< vector<Point> > outPutContours;
    for (const auto & contour : contours) {
        double area = contourArea(contour);
        if (area > minArea)
            outPutContours.push_back(contour);
    }
    return outPutContours;
}

vector< vector<Point> > getContours(Mat img) {
    /* processing image:
     *  1.convert to greyscale and blurring
     *  2.edge extraction
     *  3.blurring
     *  4.binarizing
     *  5.find contours
     *  6.find largest contour
     *  7.contour to polyline(corners get!) */
    Mat imgGrey, imgBlur, imgFilter2d, imgBlur2, imgBin;
    // 1.convert to greyscale and blurring
//    imshow("0", img);
//    imwrite("0.png", img);
    cvtColor(img, imgGrey, CV_BGR2GRAY);
//    imshow("1", imgGrey);
//    imwrite("1.png", imgGrey);
    GaussianBlur(imgGrey, imgBlur, Size(9, 9), 2);
//    imshow("2", imgBlur);
//    imwrite("2.png", imgBlur);

    // 2.edge extraction
//    Mat kernel = (Mat_<char>(3, 3) << 0, 1, 0, 1, -4, 1, 0, 1, 0); // Laplacian filter
    Mat kernel = (Mat_<char>(3, 3) << -1, -1, -1, -1, 8, -1, -1, -1, -1); // Laplacian filter
//    Mat kernel = (Mat_<char>(3, 3) << -1, -2, -1, 0, 0, 0, 1, 2, 1); // Sobel filter x_dir
    filter2D(imgBlur, imgFilter2d, -1, kernel);

//    Mat m3 = imgFilter2d.clone(), m4;
//    normalize(m3, m3, 255, 0, NORM_MINMAX);
//    imshow("3", m3);
//    imwrite("3.png", m3);
//    blur(m3, m4, Size(9, 9));
//    imshow("4", m4);
//    imwrite("4.png", m4);
    // 3.blurring
    blur(imgFilter2d, imgBlur2, Size(9, 9));
    // 4.binarizing
    threshold(imgBlur2, imgBin, 3, 50, THRESH_BINARY);
//    imshow("5", imgBin);
//    imwrite("5.png", imgBin);
    // 5.find contours
    vector<vector<Point> > contours;
    findContours(imgBin, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

//    Mat m6 = imgBin.clone();
//    Mat m7 = imgBin.clone();
//    drawContours(m6, contours, -1, Scalar(255, 255, 255));
//    imshow("6", m6);
//    imwrite("6.png", m6);

//    cout << contours.size() << endl;
    // 6.find largest contour
    vector< vector<Point> > largeContours = findLargeContours(contours, 500.);
//    cout << largeContours.size() << endl;
    // 7.contour to polyline(corners get!)
    for (auto & largeContour : largeContours)
        approxPolyDP(largeContour, largeContour, 13, true);

//    drawContours(m7, largeContours, -1, Scalar(255, 255, 255));
//    imshow("7", m7);
//    imwrite("7.png", m7);

//    for (int i = 0; i < largeContours.size(); ++i)
//        cout << largeContours[i] << endl;
//    drawContours(img, largeContours, -1, Scalar(0, 0, 255));
//
//    imshow("myImage", img);
//    imshow("filter2d", imgFilter2d);
//    waitKey(0);
    return largeContours;
}

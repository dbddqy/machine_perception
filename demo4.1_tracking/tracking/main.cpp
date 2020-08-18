//
// Created by yue on 18.02.20.
//

#include <iostream>
#include <boost/format.hpp>  // for formating strings
#include <bitset>

#include <opencv2/aruco/dictionary.hpp>

using namespace std;

int main(int argc, char **argv) {
//    if (argc != 2) {
//        cout << "please enter the file path." << endl;
//        return -1;
//    }
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    cv::Mat store=dictionary->bytesList;
//    cv::FileStorage fs("dic_save.yml", cv::FileStorage::WRITE);
//    fs << "MarkerSize" << dictionary->markerSize;
//    fs << "MaxCorrectionBits" << dictionary->maxCorrectionBits;
//    fs << "ByteList" << dictionary->bytesList;
//    fs.release();

    for (int i = 0; i < store.rows; ++i) {
        cout << bitset<8>(store.at<uint8_t >(i, 0))
             << bitset<8>(store.at<uint8_t >(i, 1)) << endl;
    }

    cout << "hello world." << endl;
    return 0;
}
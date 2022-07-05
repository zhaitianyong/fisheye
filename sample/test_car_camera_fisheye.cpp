//
// Created by atway on 2022/7/5.
//


#include <iostream>
#include "opencv_fisheye.h"
int main(){
    using namespace std;

    std::vector<cv::String> files;
    string pattern="/home/atway/Datasets/环视/left_surround_190/*.bmp";
    cv::glob(pattern, files, false);

    cv::Mat cameraMatrix, distCoeffs;
    cv::Size boardSize(11, 8);
    cv::Size2f squareSize(90., 90.);
    runCalibration(files, boardSize, squareSize, cameraMatrix,distCoeffs, true);
    std::cout << "cameraMatrix:" << cameraMatrix << endl;
    std::cout << "distCoeffs:" << distCoeffs << endl;

    undistImages(files, cameraMatrix, distCoeffs, true);


    return 0;
}
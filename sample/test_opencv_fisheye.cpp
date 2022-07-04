//
// Created by atway on 2022/7/4.
//

#include <iostream>
#include "opencv_fisheye.h"
int main(){
    using namespace std;

    std::vector<cv::String> files;
    string pattern="../data/calib_images/left_*.jpg";
    cv::glob(pattern, files, false);

    cv::Mat cameraMatrix, distCoeffs;

    runCalibration(files,cameraMatrix,distCoeffs, true);
    std::cout << "cameraMatrix:" << cameraMatrix << endl;
    std::cout << "distCoeffs:" << distCoeffs << endl;

    undistImages(files, cameraMatrix, distCoeffs);


    return 0;
}
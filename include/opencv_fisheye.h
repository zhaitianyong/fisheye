//
// Created by atway on 2022/7/4.
//

#ifndef FISHEYE_OPENCV_FISHEYE_H
#define FISHEYE_OPENCV_FISHEYE_H

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

void runCalibration(std::vector<cv::String>& files, cv::Size& boardSize,cv::Size2f& squareSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, bool fisheye);
void undistImages(std::vector<cv::String>& files,const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs, bool fisheye);

#endif //FISHEYE_OPENCV_FISHEYE_H

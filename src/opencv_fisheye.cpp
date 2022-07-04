//
// Created by atway on 2022/7/4.
//
#include "opencv_fisheye.h"

using namespace Eigen;
using namespace cv;
using namespace std;

/**
 * 旋转矩阵转换为旋转向量
 */
Vector3d rotationMatrix2Vector(const Matrix3d &R) {

    AngleAxisd r;
    r.fromRotationMatrix(R);
    return r.angle() * r.axis();
}

/**
 *
 * 旋转向量到旋转矩阵
 */

Matrix3d rotationVector2Matrix(const Vector3d &v) {
    double s = sqrt(v.dot(v));
    Vector3d axis = v / s;
    AngleAxisd r(s, axis);
    return r.toRotationMatrix();
}


void getObjecPoints(const cv::Size &borderSize, const cv::Size2f &squareSize, std::vector<Point3f> &objectPoints) {

    for (int r = 0; r < borderSize.height; ++r) {
        for (int c = 0; c < borderSize.width; ++c) {

            objectPoints.push_back(Point3f(c * squareSize.width, r * squareSize.height, 0.));
        }

    }
}


double computeReprojectionErrors(const vector<vector<Point3f> > &objectPoints,
                                 const vector<vector<Point2f> > &imagePoints,
                                 const vector<Mat> &rvecs, const vector<Mat> &tvecs,
                                 const Mat &cameraMatrix, const Mat &distCoeffs,
                                 vector<float> &perViewErrors, bool fisheye) {
    vector<Point2f> imagePoints2;
    size_t totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        if (fisheye) {
            fisheye::projectPoints(objectPoints[i], imagePoints2, rvecs[i], tvecs[i], cameraMatrix, distCoeffs);
        } else {
            projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        }
        err = norm(imagePoints[i], imagePoints2, NORM_L2);

        size_t n = objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

void runCalibration(std::vector<cv::String> &files, Mat &cameraMatrix, Mat &distCoeffs, bool fisheye) {

    std::vector<std::vector<Point2f>> imagePoints;
    vector<vector<cv::Point3f>> objectPoints;
    cv::Size boardSize(8, 6);
    cv::Size2f squareSize(20., 20.);
    cv::Size imageSize;
    for (int i = 0; i < files.size(); ++i) {

        cv::Mat img = cv::imread(files[i]);
        std::vector<cv::Point2f> corners;
        if (i == 0) imageSize = img.size();
        bool ok = cv::findChessboardCorners(img, boardSize, corners,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
                                            cv::CALIB_CB_NORMALIZE_IMAGE);
        if (ok) {
            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001));
            imagePoints.push_back(corners);

            cv::drawChessboardCorners(img, boardSize, cv::Mat(corners), ok);
            cv::imshow("corners", img);
            cv::waitKey(500);
        }
    }

    for (int i = 0; i < imagePoints.size(); ++i) {
        std::vector<Point3f> corners;
        getObjecPoints(boardSize, squareSize, corners);
        objectPoints.push_back(corners);
    }

//    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    double res = 0;
    if (fisheye) {
        int flag = 0;
        flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
        //flag |= cv::fisheye::CALIB_CHECK_COND;
        flag |= cv::fisheye::CALIB_FIX_SKEW;
        //flag |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
        distCoeffs = Mat::zeros(4, 1, CV_64F);
        res = cv::fisheye::calibrate(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs,
                                     flag);
    } else {
        res = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    }

    vector<float> perViewErrors;
    double totalError = computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeffs,
                                                  perViewErrors, fisheye);

    std::cout << "total error :" << totalError << std::endl;
    for (int j = 0; j < perViewErrors.size(); ++j) {
        std::cout << j << " : " << perViewErrors[j] << std::endl;
    }


}


void undistImages(std::vector<cv::String> &files, const Mat &cameraMatrix, const Mat &distCoeffs) {


    for (int i = 0; i < files.size(); ++i) {
        cv::Mat img = cv::imread(files[i]);
        Mat map1, map2, undistImg;
        Mat newCameraMatrix;
        Matx33d R = cv::Matx33d::eye();
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, img.size(), R,
                                                                newCameraMatrix, 1.0);
        cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Matx33d::eye(), newCameraMatrix, img.size(),
                                             CV_16SC2, map1, map2);
        cv::remap(img, undistImg, map1, map2, INTER_LINEAR);

        cv::imshow("undist", undistImg);
        cv::waitKey(500);
    }
}
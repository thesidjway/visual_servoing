#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>

struct PoseEstimationParams {
    PoseEstimationParams() {
        K.at<double> ( 0,0 ) = fx;
        K.at<double> ( 0,2 ) = cx;
        K.at<double> ( 1,1 ) = fy;
        K.at<double> ( 1,2 ) = cy;
    }
    double fx = 300;
    double fy = 300;
    double cx = 160;
    double cy = 120;
    cv::Mat K = cv::Mat ( 3, 3, CV_64F );
    double marker_length = 0.1;
};

class VisualServoing {
public:
    VisualServoing();
    ~VisualServoing();
    void detectArucoTags ( cv::Mat &img, Eigen::Vector4d  &marker_point, Eigen::Vector2d &marker_projection );
    void readDetectorParameters ( std::string filename );
private:
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    PoseEstimationParams pose_estimation_params_;
};
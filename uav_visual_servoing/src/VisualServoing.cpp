#include <uav_visual_servoing/VisualServoing.h>

VisualServoing::VisualServoing() {
    detector_params_ = cv::aruco::DetectorParameters::create();
}

VisualServoing::~VisualServoing() {
}


void VisualServoing::readDetectorParameters ( std::string params_file ) {
    cv::FileStorage fs ( params_file, cv::FileStorage::READ );
    if ( !fs.isOpened() ) {
        exit ( EXIT_FAILURE );
    }
    fs["adaptiveThreshWinSizeMin"] >> detector_params_->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> detector_params_->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> detector_params_->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> detector_params_->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> detector_params_->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> detector_params_->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> detector_params_->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> detector_params_->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> detector_params_->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> detector_params_->minMarkerDistanceRate;
    fs["cornerRefinementWinSize"] >> detector_params_->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> detector_params_->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> detector_params_->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> detector_params_->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> detector_params_->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> detector_params_->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> detector_params_->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> detector_params_->minOtsuStdDev;
    fs["fx"] >> pose_estimation_params_.fx;
    fs["fy"] >> pose_estimation_params_.fy;
    fs["cx"] >> pose_estimation_params_.cx;
    fs["cy"] >> pose_estimation_params_.cy;
    pose_estimation_params_.K.at<double> ( 0,0 ) = pose_estimation_params_.fx;
    pose_estimation_params_.K.at<double> ( 0,2 ) = pose_estimation_params_.cx;
    pose_estimation_params_.K.at<double> ( 1,1 ) = pose_estimation_params_.fy;
    pose_estimation_params_.K.at<double> ( 1,2 ) = pose_estimation_params_.cy;
    fs["markerLength"] >> pose_estimation_params_.marker_length;
}



void VisualServoing::detectArucoTags ( cv::Mat &img, Eigen::Vector4d &marker_point, Eigen::Vector2d &marker_projection ) {
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary ( 8 ); //DICT_6X6_50
    cv::aruco::detectMarkers ( img,
                               dictionary,
                               markerCorners,
                               markerIds,
                               detector_params_,
                               cv::noArray() );
    cv::aruco::drawDetectedMarkers ( img, markerCorners, markerIds );
    cv::Mat r_marker, t_marker, r_marker_mat;
    Eigen::Matrix<double, 3,1> et_marker;
    Eigen::Matrix3d er_marker_mat;
    cv::aruco::estimatePoseSingleMarkers( markerCorners, pose_estimation_params_.marker_length, pose_estimation_params_.K, cv::noArray(), r_marker, t_marker );
    if ( t_marker.rows > 0 ) {
//         std::cout << t_marker.at<double> ( 0 , 0 ) << " " <<  t_marker.at<double> ( 0 , 1 ) << " " << t_marker.at<double> ( 0 , 2 ) << std::endl;
        marker_point = Eigen::Vector4d ( t_marker.at<double> ( 0,0 ) , t_marker.at<double> ( 0 , 1 ) , t_marker.at<double> ( 0 , 2 ), r_marker.at<double>(0,2)); // x, y, z, yaw
        marker_projection ( 0 ) = ( markerCorners[0][0].x + markerCorners[0][1].x + markerCorners[0][2].x + markerCorners[0][3].x ) / 4 ;
        marker_projection ( 1 ) = ( markerCorners[0][0].y + markerCorners[0][1].y + markerCorners[0][2].y + markerCorners[0][3].y ) / 4 ;
    }
}

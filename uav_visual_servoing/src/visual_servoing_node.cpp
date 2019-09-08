#include <uav_visual_servoing/visual_servoing_node.h>

visual_servoing_node::visual_servoing_node() {
    std::string aruco_param_file;
    std::string camera_topic, odom_topic;
    nh_.param<std::string>("aruco_params", aruco_param_file, "/home/thesidjway/icra_ws/src/visual_servoing/uav_visual_servoing/params/aruco_params.yaml");
    nh_.param<std::string>("camera_topic", camera_topic, "/baroness/downward/camera_/image_raw");
    nh_.param<std::string>("odom_topic", odom_topic, "/baroness/odometry_sensor1/odometry");
    detector_.readDetectorParameters(aruco_param_file);
    camera_sub_ =  nh_.subscribe(camera_topic, 1, &visual_servoing_node::imageCallback, this);
    odom_sub_ =  nh_.subscribe(odom_topic, 1, &visual_servoing_node::odomCallback, this);
    trajectory_pub_ = nh_.advertise <trajectory_msgs::MultiDOFJointTrajectory> (mav_msgs::default_topics::COMMAND_TRAJECTORY, 5);
}

void visual_servoing_node::odomCallback(const nav_msgs::Odometry_< std::allocator< void > >::ConstPtr odom_msg) {
    last_pose_ = odom_msg->pose.pose;
}

static geometry_msgs::Quaternion createQuaternionFromRPY(double roll, double pitch, double yaw) {
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code
    // http://docs.ros.org/api/geometry_msgs/html/msg/Quaternion.html
    geometry_msgs::Quaternion q;
    double t0 = cos(yaw * 0.5);
    double t1 = sin(yaw * 0.5);
    double t2 = cos(roll * 0.5);
    double t3 = sin(roll * 0.5);
    double t4 = cos(pitch * 0.5);
    double t5 = sin(pitch * 0.5);
    q.w = t0 * t2 * t4 + t1 * t3 * t5;
    q.x = t0 * t3 * t4 - t1 * t2 * t5;
    q.y = t0 * t2 * t5 + t1 * t3 * t4;
    q.z = t1 * t2 * t4 - t0 * t3 * t5;
    return q;
}


void visual_servoing_node::imageCallback(const sensor_msgs::Image::ConstPtr image_msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat read_image_distorted = cv_ptr->image;
        cv::cvtColor(read_image_distorted, read_image_distorted, CV_BGR2GRAY);
        cv::Mat read_image;
        read_image = read_image_distorted;
        Eigen::Vector4d pt;
        Eigen::Vector2d proj(-10000, -10000);
        detector_.detectArucoTags(read_image, pt, proj);
        if (proj.norm() < 10000) {
            //Based on 4DOF assumption
            geometry_msgs::Pose odom_new;
            odom_new.position.x = last_pose_.position.x - pt(1);
            odom_new.position.y = last_pose_.position.y - pt(0);
            double roll, pitch, yaw;
            tf::Quaternion q(last_pose_.orientation.x, last_pose_.orientation.y, last_pose_.orientation.z, last_pose_.orientation.w);
            tf::Matrix3x3 quaternion(q);
            quaternion.getRPY(roll, pitch, yaw);
            mav_msgs::EigenTrajectoryPoint trajectory_point;
            trajectory_msgs::MultiDOFJointTrajectory samples_array;
            trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;
            double x_target, y_target, z_target;
            trajectory_point.position_W.x() = odom_new.position.x;
            trajectory_point.position_W.y() = odom_new.position.y;
            trajectory_point.position_W.z() = 2;
            trajectory_point.setFromYaw(yaw);
            samples_array.points.clear();
            tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0);
            trajectory_point.setFromYaw(tf::getYaw(quat));
            mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
            samples_array.points.push_back(trajectory_point_msg);
            trajectory_pub_.publish(samples_array);
            std::cout <<odom_new.position.x << " " << odom_new.position.y << std::endl;
        }
        cv::imshow("Read Image", read_image);
        cv::waitKey(1);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


visual_servoing_node::~visual_servoing_node() {
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_visual_servoing");
    ros::NodeHandle nh;
    visual_servoing_node vsn;
    ROS_INFO("Started Servoing");
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;

    // Trying to unpause Gazebo for 10 seconds.
    while (i <= 10 && !unpaused) {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        ++i;
    }

    if (!unpaused) {
        ROS_FATAL("Could not wake up Gazebo.");
        return -1;
    } else {
        ROS_INFO("Unpaused the Gazebo simulation.");
    }

    trajectory_msgs::MultiDOFJointTrajectory samples_array;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg;

    // Wait for 6 seconds to let the Gazebo GUI show up.
    ros::Duration(6.0).sleep();

    // This is the initialization motion, necessary that the known free space allows the planning
    // of initial paths.
    double x_target, y_target, z_target;
    ROS_INFO("Taking Off");
    nh.param<double>("wp_x", x_target, 0.0);
    nh.param<double>("wp_y", y_target, 0.0);
    nh.param<double>("wp_z", z_target, 1.0);
    trajectory_point.position_W.x() = x_target - 0.3;
    trajectory_point.position_W.y() = y_target;
    trajectory_point.position_W.z() = z_target;


    for (double i = 0; i <= 1.0; i = i + 0.1) {
        trajectory_point.position_W.z() = z_target * 0.25 + z_target * 0.15 * i;
        samples_array.header.stamp = ros::Time::now();
        samples_array.points.clear();
        tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0);
        trajectory_point.setFromYaw(tf::getYaw(quat));
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        samples_array.points.push_back(trajectory_point_msg);
        vsn.trajectory_pub_.publish(samples_array);
        ros::Duration(0.5).sleep();
    }

    for (double i = 0; i <= 1.0; i = i + 0.1) {
        trajectory_point.position_W.x() = x_target - 0.3;
        trajectory_point.position_W.y() = y_target;
        trajectory_point.position_W.z() = z_target * 0.4 + i * z_target * 0.6;
        samples_array.header.stamp = ros::Time::now();
        samples_array.points.clear();
        tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), 0);
        trajectory_point.setFromYaw(tf::getYaw(quat));
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(trajectory_point, &trajectory_point_msg);
        samples_array.points.push_back(trajectory_point_msg);
        vsn.trajectory_pub_.publish(samples_array);
        ros::Duration(0.5).sleep();
    }

    while (ros::ok()) {
        ros::spinOnce();
    }
    return -1;
}








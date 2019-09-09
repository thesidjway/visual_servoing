#pragma once
#include <uav_visual_servoing/VisualServoing.h>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <eigen_conversions/eigen_msg.h>

class visual_servoing_node {
public:
    visual_servoing_node();
    ~visual_servoing_node();
    ros::Publisher trajectory_pub_; 
private:
    VisualServoing detector_;
    ros::NodeHandle nh_;
    ros::Subscriber camera_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber husky_odom_sub_;
    geometry_msgs::Pose last_pose_;
    geometry_msgs::Pose last_husky_pose_;
    void imageCallback(const sensor_msgs::Image::ConstPtr image_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr odom_msg);
    void huskyOdomCallback(const nav_msgs::Odometry::ConstPtr odom_msg);
};
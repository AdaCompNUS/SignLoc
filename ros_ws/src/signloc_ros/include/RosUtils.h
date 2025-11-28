/**
# ##############################################################################
#  Copyright (c) 2024- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: RosUtils.h                                                         #
# ##############################################################################
**/

#ifndef ROSUTILS_H
#define ROSUTILS_H

#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <vector>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>



Eigen::Vector4d OdomMsg2Pose(const nav_msgs::OdometryConstPtr& odom);

// Eigen::Vector4d PoseMsg2Pose2D(const geometry_msgs::PoseStampedConstPtr& poseMsg);

// std::vector<float> OdomMsg2Pose3D(const nav_msgs::OdometryConstPtr& odom);

// std::vector<float> PoseMsg2Pose3D(const geometry_msgs::PoseStampedConstPtr& odom);

// geometry_msgs::PoseStamped Pose2D2PoseMsg(Eigen::Vector3d pose2d);

// sensor_msgs::Image CVMat2ImgMsg(const cv::Mat& img, std_msgs::Header header, const std::string& type);

// geometry_msgs::PoseWithCovarianceStamped Pred2PoseWithCov(Eigen::Vector3d pred, Eigen::Matrix3d cov);


#endif

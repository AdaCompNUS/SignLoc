/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: RosUtils.h                                                        #
# ##############################################################################
**/


#ifndef ROSUTILS_H
#define ROSUTILS_H

#include <eigen3/Eigen/Dense>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>


Eigen::Vector4d OdomMsg2Pose(const nav_msgs::msg::Odometry::ConstSharedPtr& odom);


Eigen::Vector3f OdomMsg2Pose2D(const nav_msgs::msg::Odometry::ConstSharedPtr& odom);

Eigen::Vector3f PoseMsg2Pose2D(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& poseMsg);

std::vector<float> OdomMsg2Pose3D(const nav_msgs::msg::Odometry::ConstSharedPtr& odom);

std::vector<float> PoseMsg2Pose3D(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& odom);

geometry_msgs::msg::PoseStamped Pose2D2PoseMsg(Eigen::Vector3f pose2d);


geometry_msgs::msg::PoseWithCovarianceStamped Pred2PoseWithCov(Eigen::Vector3d pred, Eigen::Matrix3d cov);

#endif
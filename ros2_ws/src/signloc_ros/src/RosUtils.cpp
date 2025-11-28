/**
# ##############################################################################
#  Copyright (c) 2021- University of Bonn                                      #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: RosUtils.cpp                                                          #
# ##############################################################################
**/


#include "RosUtils.h"
#include "Utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
//#include <cv_bridge/cv_bridge.h>


Eigen::Vector4d OdomMsg2Pose(const nav_msgs::msg::Odometry::ConstSharedPtr& odom)
{
    float x = odom->pose.pose.position.x;
    float y = odom->pose.pose.position.y;
    float z = odom->pose.pose.position.z;
    float qz = odom->pose.pose.orientation.z;
    float qw = odom->pose.pose.orientation.w;

    Eigen::Vector4d pose = Eigen::Vector4d(x, y, z, GetYaw(qz, qw));

    return pose;
}



geometry_msgs::msg::PoseStamped Pose2D2PoseMsg(Eigen::Vector3f pose2d)
{
    geometry_msgs::msg::PoseStamped poseStamped;
    poseStamped.pose.position.x = pose2d(0);
    poseStamped.pose.position.y = pose2d(1);
    poseStamped.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0.0,0.0, pose2d(2));
    q = q.normalize();

    poseStamped.pose.orientation.x = q[0];
    poseStamped.pose.orientation.y = q[1];
    poseStamped.pose.orientation.z = q[2];
    poseStamped.pose.orientation.w = q[3];

    return poseStamped;
}


Eigen::Vector3f OdomMsg2Pose2D(const nav_msgs::msg::Odometry::ConstSharedPtr& odom)
{
    float x = odom->pose.pose.position.x;
    float y = odom->pose.pose.position.y;
    float qz = odom->pose.pose.orientation.z;
    float qw = odom->pose.pose.orientation.w;

    Eigen::Vector3f pose = Eigen::Vector3f(x, y, GetYaw(qz, qw));

    return pose;
}

Eigen::Vector3f PoseMsg2Pose2D(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& poseMsg)
{
    float x = poseMsg->pose.position.x;
    float y = poseMsg->pose.position.y;
    float qz = poseMsg->pose.orientation.z;
    float qw = poseMsg->pose.orientation.w;

    Eigen::Vector3f pose = Eigen::Vector3f(x, y, GetYaw(qz, qw));

    return pose;
}

std::vector<float> OdomMsg2Pose3D(const nav_msgs::msg::Odometry::ConstSharedPtr& odom)
{

    std::vector<float> pose;
    pose.push_back(odom->pose.pose.position.x);
    pose.push_back(odom->pose.pose.position.y);
    pose.push_back(odom->pose.pose.position.y);
    pose.push_back(odom->pose.pose.orientation.x);
    pose.push_back(odom->pose.pose.orientation.y);
    pose.push_back(odom->pose.pose.orientation.z);
    pose.push_back(odom->pose.pose.orientation.w);
    
    return pose;

}

std::vector<float> PoseMsg2Pose3D(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& odom)
{

    std::vector<float> pose;
    pose.push_back(odom->pose.position.x);
    pose.push_back(odom->pose.position.y);
    pose.push_back(odom->pose.position.y);
    pose.push_back(odom->pose.orientation.x);
    pose.push_back(odom->pose.orientation.y);
    pose.push_back(odom->pose.orientation.z);
    pose.push_back(odom->pose.orientation.w);
    
    return pose;
}





geometry_msgs::msg::PoseWithCovarianceStamped Pred2PoseWithCov(Eigen::Vector3d pred, Eigen::Matrix3d cov)
{

    geometry_msgs::msg::PoseWithCovarianceStamped poseStamped;
 
    poseStamped.pose.pose.position.x = pred(0);
    poseStamped.pose.pose.position.y = pred(1);
    poseStamped.pose.pose.position.z = 0.1;
    tf2::Quaternion q;
    q.setRPY(0.0,0.0, pred(2));
    q = q.normalize();

    poseStamped.pose.pose.orientation.x = q[0];
    poseStamped.pose.pose.orientation.y = q[1];
    poseStamped.pose.pose.orientation.z = q[2];
    poseStamped.pose.pose.orientation.w = q[3];

    // float covariance[36] = {0};

    // // x-x
    // covariance[0] = cov(0, 0);
    // // x-y
    // covariance[1] = cov(0, 1);
    // // y-x
    // covariance[6] = cov(1, 0);
    // // y-y
    // covariance[7] = cov(1, 1);
    // // yaw-yaw
    // covariance[35] = cov(2, 2);

    poseStamped.pose.covariance[0] = cov(0, 0);
    poseStamped.pose.covariance[1] = cov(0, 1);
    poseStamped.pose.covariance[6] = cov(1, 0);
    poseStamped.pose.covariance[7] = cov(1, 1);
    poseStamped.pose.covariance[35] = cov(2, 2);


    return poseStamped;
}


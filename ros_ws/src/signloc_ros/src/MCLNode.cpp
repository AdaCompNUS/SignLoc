/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MCLNode.cpp                                                           #
# ##############################################################################
**/

 
#include <eigen3/Eigen/Dense>
#include "RosUtils.h"
#include "MCL.h"
#include "Utils.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>  
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2_ros/transform_listener.h>  
#include <tf/transform_broadcaster.h>
#include <chrono>

#include "signloc_msgs/UTM.h"
#include "signloc_msgs/Sign.h"
#include "signloc_msgs/SignElement.h"

  

class MCLNode
{
public:
  MCLNode()  
  {
    ros::NodeHandle nh;

    int numParticles;
    std::string odomTopic;
    std::string signTopic;
    std::string poseTopic;
    std::string navGraphPath;
    std::string utmTopic;
    std::string particleTopic;



    nh.getParam("odomTopic", odomTopic); 
    ROS_INFO_STREAM("MCLNode::odomTopic " << odomTopic);
    nh.getParam("signTopic", signTopic); 
    ROS_INFO_STREAM("MCLNode::signTopic " << signTopic);
    nh.getParam("poseTopic", poseTopic);
    ROS_INFO_STREAM("MCLNode::poseTopic " << poseTopic);
    nh.getParam("utmTopic", utmTopic);
    ROS_INFO_STREAM("MCLNode::utmTopic " << utmTopic);
    nh.getParam("particleTopic", particleTopic);
    ROS_INFO_STREAM("MCLNode::particleTopic " << particleTopic);
    nh.getParam("navGraphPath", navGraphPath); 
    ROS_INFO_STREAM("MCLNode::navGraphPath " << navGraphPath); 


    while (ros::ok())
    {
      boost::shared_ptr<const signloc_msgs::UTM> msg = ros::topic::waitForMessage<signloc_msgs::UTM>(utmTopic);
      if (msg != nullptr)
      {
          o_gpsOffset = Eigen::Vector2d(msg->lat, msg->lon); 
          break;
      }
    }

    ROS_INFO_STREAM("MCLNode:: gps center " << o_gpsOffset(0) << ", " << o_gpsOffset(1));  

    //o_gpsOffset = Eigen::Vector2d(363784.0139568621, 143122.7189409806);
    o_mapTopic = "map";
    o_mtx = new std::mutex();  
   
    o_mcl = std::make_shared<MCL>(MCL(navGraphPath, -1, o_gpsOffset, true, 0.5));  

    o_odomSub = nh.subscribe(odomTopic, 1, &MCLNode::motionCallback, this);   
    o_signSub = nh.subscribe(signTopic, 1, &MCLNode::observationCallback, this);  
    o_armSub = nh.subscribe("/spot/gripper_cam_pose", 1, &MCLNode::armCallback, this);     
    o_particlePub = nh.advertise<geometry_msgs::PoseArray>(particleTopic, 10);
    o_markerPub = nh.advertise<visualization_msgs::Marker>("/MCLPrediction", 1); 


    ROS_INFO_STREAM("MCLNode::Ready!");
  }

  void armCallback(const std_msgs::StringConstPtr& arm_msg)
  {
    if (arm_msg->data != "none")
    {
      o_mtx->lock();
      o_armStatus = arm_msg->data;
      ROS_INFO_STREAM("MCLNode:: arm status " << o_armStatus);
      o_mtx->unlock();
    } 

  }

  void motionCallback(const nav_msgs::OdometryConstPtr& odom_msg)
  {

    if (!o_initObservation) return;
    if (!o_initOdom)
    {
      o_initOdom = true;
      o_prevPose = OdomMsg2Pose(odom_msg);   
      ROS_INFO_STREAM("MCLNode::InitOdom");
      return;  
    }

    Eigen::Vector4d currPose = OdomMsg2Pose(odom_msg);
    Eigen::Vector4d delta = currPose - o_prevPose; 

    if(((delta.norm() > o_triggerDist) || (fabs(delta(3)) > o_triggerAngle)))
    {

          //ROS_INFO_STREAM("MCLNode::Predict!");
        Eigen::Vector4d u = o_mcl->Backward(o_prevPose, currPose);
        o_mtx->lock();
        o_mcl->Predict(u, o_odomNoise);
        publishParticles(odom_msg->header.stamp);
        o_mtx->unlock(); 

        o_prevPose = currPose;

    }
  }
 
  void observationCallback(const signloc_msgs::SignConstPtr& sign_msg)
  {

    std::unordered_map<std::string, int > actionDict = {{"RIGHT", 0}, {"STRAIGHT-RIGHT",1}, {"STRAIGHT", 2}, {"STRAIGHT-LEFT", 3}, {"LEFT", 4}, {"BACK-LEFT", 5}, {"BACK", 6}, {"BACK-RIGHT", 7}, {"LOCATIONAL", 8}, {"UP", 9}, {"DOWN", 10}};
    std::unordered_map<std::string, std::vector<std::tuple<int, double>>> minigraph;
    std::vector<signloc_msgs::SignElement> elements = sign_msg->elements;

    o_mtx->lock();
    std::string armStatus = o_armStatus;
    o_mtx->unlock(); 

    for(int i = 0; i < elements.size(); ++i)
    {
        std::string place = elements[i].place;
        std::vector<std::string> directions = elements[i].directions;
        std::vector<double> probs = elements[i].probs;  

        if (place == "TOILETS") continue;

        std::vector<std::tuple<int, double>> actionDist;
        ROS_INFO_STREAM("Place label " + place); 
        for(int d = 0; d < directions.size(); ++d)
        {
            std::string direction = directions[d]; 
            int action  = actionDict[direction]; 
            double prob = probs[d];

            if (action >= 8) break;

            if (armStatus == "right")
            {
               action = (action + 6) % 8; 
            }
            else if (armStatus == "left")
            {
               action = (action + 2) % 8;
            }
            
            ROS_INFO_STREAM("Action " + std::to_string(action) + " Prob " + std::to_string(prob)); 

            std::tuple<int, double> actionAndProb = std::make_tuple(action, prob);
            actionDist.push_back(actionAndProb);
        }

        if (actionDist.size()) minigraph[place] = actionDist;
    }



    o_mtx->lock();
    o_mcl->Correct(minigraph);
    std::vector<Particle> particles = o_mcl->Particles(); 
    std::sort(particles.begin(), particles.end(), [](const Particle& p1, const Particle& p2) { 
        return p1.Weight() > p2.Weight();  // Returns true if 'a' should come before 'b' (for descending)
    });

    //Eigen::Vector4d pose = particles[0].Pose();
    int id = particles[0].ID(); 
    Eigen::Vector3d pos = o_mcl->Graph()->Node(id)->Pos();
    Eigen::Vector4d pose(pos(0), pos(1), pos(2), particles[0].Pose()(3));


    for (int i = 0; i < 5; ++i)
    {
      ROS_INFO_STREAM( i <<"th best particle is (" << particles[i].ID() << "," << particles[i].Pose()(3) << ") , with weight " << particles[i].Weight());
    }
    o_mcl->Resampling();

    publishPrediction(sign_msg->header.stamp, pose);
    publishParticles(sign_msg->header.stamp); 
    o_armStatus = "none";
    ROS_INFO_STREAM("MCLNode:: arm status " << o_armStatus);

    //ROS_INFO_STREAM("MCLNode:: best node " << o_armStatus);


    o_mtx->unlock();  
    o_initObservation = true;
    ROS_INFO_STREAM("MCLNode::Sign!");  
    return; 
  }

  void publishPrediction(ros::Time stamp, Eigen::Vector4d pose)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = o_mapTopic;
    marker.header.stamp = stamp;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose(0) - o_gpsOffset(0);
    marker.pose.position.y = pose(1) - o_gpsOffset(1); 
    marker.pose.position.z = 5 * pose(2);
    tf2::Quaternion q;
    q.setRPY( 0, 0, 0); 
    marker.pose.orientation.x = q[0]; 
    marker.pose.orientation.y = q[1]; 
    marker.pose.orientation.z = q[2];
    marker.pose.orientation.w = q[3];
    marker.scale.x = 3;
    marker.scale.y = 3;
    marker.scale.z = 3;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(500);
    o_markerPub.publish(marker); 
  }


  void publishParticles(ros::Time stamp)
  {
    geometry_msgs::PoseArray  posearray;
    posearray.header.stamp = stamp;  
    posearray.header.frame_id = o_mapTopic;

    std::vector<Particle> particles = o_mcl->Particles();   

    for (int i = 0; i < particles.size(); ++i)
    {
        geometry_msgs::Pose p;
        Eigen::Vector4d pose = particles[i].Pose();
        //if (particles[i].Weight() < 0.01) continue;
        p.position.x = pose(0) - o_gpsOffset(0); 
        p.position.y = pose(1) - o_gpsOffset(1) ;
        p.position.z = 5 * pose(2); 
        tf2::Quaternion q;
        q.setRPY( 0, 0, pose(3)); 
        p.orientation.x = q[0];
        p.orientation.y = q[1];
        p.orientation.z = q[2];
        p.orientation.w = q[3];
        posearray.poses.push_back(p);
    }
    o_particlePub.publish(posearray);
  }

private:

  int count = 0;
  ros::Publisher o_posePub;
  ros::Publisher o_particlePub;
  ros::Publisher o_markerPub;
  ros::Subscriber o_odomSub;
  ros::Subscriber o_signSub;
  ros::Subscriber o_armSub;
  Eigen::Vector4d o_prevPose = Eigen::Vector4d(0, 0, 0, 0);
  Eigen::Vector4d o_prevTriggerPose = Eigen::Vector4d(0, 0, 0, 0);
  std::string o_armStatus = "none";

  Eigen::Vector4d o_odomNoise = Eigen::Vector4d(0.02, 0.02, 0.02, 0.02);
  double o_triggerDist = 0.1;
  double o_triggerAngle = 0.1 * M_PI;
  std::string o_mapTopic;
  std::string o_baseLinkTF;
  bool o_initOdom = false;
  bool o_initObservation = false;
  std::mutex* o_mtx;
  std::shared_ptr<MCL> o_mcl;
 
  Eigen::Vector2d o_gpsOffset;



};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MCLNode");
  MCLNode mcl = MCLNode();
  ros::spin();
  

  return 0;
}

/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MCLNode.cpp                                                    #
# ##############################################################################
**/  

#include <chrono>      
#include <functional> 
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "std_msgs/msg/string.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "tf2/LinearMath/Quaternion.h"
// #include "tf2_ros/transform_broadcaster.h"
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp> 
#include <visualization_msgs/msg/marker.hpp>

#include "signloc_msgs/msg/utm.hpp"
#include "signloc_msgs/msg/sign.hpp"
#include "signloc_msgs/msg/sign_element.hpp"

#include <mutex> 
#include <sstream>  
#include <fstream>

#include <eigen3/Eigen/Dense>
#include "RosUtils.h"
#include "MCL.h"
#include "Utils.h"


class MCLNode : public rclcpp::Node
{      
  public:
    MCLNode() : Node("MCLNode")
    {

      int numParticles;
      std::string odomTopic;
      std::string signTopic;
      std::string poseTopic;
      std::string utmTopic;
      std::string particleTopic;
  

      this->declare_parameter("numParticles", 100);
      this->declare_parameter("odomTopic", "");
      this->declare_parameter("signTopic", "");
      this->declare_parameter("particleTopic", "");
      this->declare_parameter("poseTopic", "");
      this->declare_parameter("utmTopic", "");
      this->declare_parameter("navGraphPath", "");
      // this->declare_parameter("odomNoise", std::vector<double>(3)); 
     
 
      this->get_parameter("numParticles", numParticles);
      RCLCPP_INFO(this->get_logger(), "numParticles %d", numParticles);
      this->get_parameter("signTopic", signTopic);
      RCLCPP_INFO(this->get_logger(), "signTopic %s", signTopic.c_str());
      this->get_parameter("navGraphPath", o_navGraphPath);
      RCLCPP_INFO(this->get_logger(), "navGraphPath %s", o_navGraphPath.c_str());
      this->get_parameter("particleTopic", particleTopic);
      RCLCPP_INFO(this->get_logger(), "particleTopic %s", particleTopic.c_str());      
      this->get_parameter("odomTopic", odomTopic);
      RCLCPP_INFO(this->get_logger(), "odomTopic %s", odomTopic.c_str());
      this->get_parameter("poseTopic", poseTopic);
      RCLCPP_INFO(this->get_logger(), "poseTopic %s", poseTopic.c_str());
      this->get_parameter("utmTopic", utmTopic);
      RCLCPP_INFO(this->get_logger(), "utmTopic %s", utmTopic.c_str());
     
      // this->get_parameter("baseLinkTF", o_baseLinkTF);
      // RCLCPP_INFO(this->get_logger(), "baseLinkTF %s", o_baseLinkTF.c_str());
      // rclcpp::Parameter dblArrParam =this->get_parameter("odomNoise");
      // std::vector<double> odomNoise = dblArrParam.as_double_array();
      // RCLCPP_INFO(this->get_logger(), "odomNoise %f %f %f", odomNoise[0], odomNoise[1], odomNoise[2]);
      // o_odomNoise = Eigen::Vector3f(odomNoise[0], odomNoise[1], odomNoise[2]); 


      // auto utmMsg = signloc_msgs::msg::UTM();
      // while (rclcpp::ok())
      // {
      //   bool success = rclcpp::wait_for_message(utmMsg, this->shared_from_this(), utmTopic, std::chrono::seconds(1));
      //   if (success)
      //   {
      //       o_gpsOffset = Eigen::Vector2d(utmMsg.lat, utmMsg.lon); 
      //       RCLCPP_INFO(this->get_logger(), "o_gpsOffset %f %f", o_gpsOffset(0), o_gpsOffset(1));  
      //       break;
      //   }
      // }

 
      o_mapTopic = "map";
      o_mtx = new std::mutex();  

      // RCLCPP_INFO(this->get_logger(), "o_gpsOffset %f %f", o_gpsOffset(0), o_gpsOffset(1));
      // RCLCPP_INFO(this->get_logger(), "navGraphPath %s", navGraphPath.c_str());

      // o_mcl = std::make_shared<MCL>(MCL(o_navGraphPath, -1, o_gpsOffset, true, 0.5)); 
      // RCLCPP_INFO(this->get_logger(), "MCLNode::Init MCL");

      //o_gpsOffset = Eigen::Vector2d(363784.000000, 143122.718750);
      //o_mcl = std::make_shared<MCL>(MCL(o_navGraphPath, -1, o_gpsOffset, true, 0.5));  
      

      rclcpp::QoS qos(10);
      qos.best_effort();  

      o_particlePub = this->create_publisher<geometry_msgs::msg::PoseArray>(particleTopic, 10);
      o_markerPub = this->create_publisher<visualization_msgs::msg::Marker>("/MCLPrediction", 10);
      o_odomSub = create_subscription<nav_msgs::msg::Odometry>(odomTopic, qos, std::bind(&MCLNode::motionCallback, this, std::placeholders::_1));
      o_signSub = create_subscription<signloc_msgs::msg::Sign>(signTopic, qos, std::bind(&MCLNode::observationCallback, this, std::placeholders::_1));
      o_armSub = create_subscription<std_msgs::msg::String>("/spot/gripper_cam_pose", qos, std::bind(&MCLNode::armCallback, this, std::placeholders::_1));
  
      o_gpsSub = create_subscription<signloc_msgs::msg::UTM>(utmTopic, qos, std::bind(&MCLNode::gpsCallback, this, std::placeholders::_1));

      // o_tfPub =  std::make_unique<tf2_ros::TransformBroadcaster>(*this);   
 
      RCLCPP_INFO(this->get_logger(), "MCLNode::Ready!");     
    } 

    void gpsCallback(const signloc_msgs::msg::UTM::SharedPtr utm_msg)
    {
        //RCLCPP_INFO(this->get_logger(), "MCLNode::gpsCallback");

      if (!o_initGPS)
      {
        o_mtx->lock();
        o_gpsOffset = Eigen::Vector2d(utm_msg->lat, utm_msg->lon); 
        RCLCPP_INFO(this->get_logger(), "o_gpsOffset %f %f", o_gpsOffset(0), o_gpsOffset(1));
        RCLCPP_INFO(this->get_logger(), "o_navGraphPath %s", o_navGraphPath.c_str());  
        o_mcl = std::make_shared<MCL>(MCL(o_navGraphPath, -1, o_gpsOffset, true, 0.5));  
        o_initGPS = true;
        o_mtx->unlock();
        RCLCPP_INFO(this->get_logger(), "MCLNode::Init!");    
      }
    }

    void armCallback(const std_msgs::msg::String::SharedPtr arm_msg)
    {
      RCLCPP_INFO(this->get_logger(), "MCLNode::armCallback");

      if (arm_msg->data != "none")
      {
        o_mtx->lock();
        o_armStatus = arm_msg->data;
        RCLCPP_INFO(this->get_logger(), "MCLNode::arm status %s", o_armStatus.c_str());  
        o_mtx->unlock();
      } 
    }


        


    void motionCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
      //RCLCPP_INFO(this->get_logger(), "MCLNode::motionCallback");

      if (!o_initGPS) return;
      if (!o_initObservation) return;
      if (!o_initOdom)
      {
        o_initOdom = true;
        o_prevPose = OdomMsg2Pose(odom_msg);   
        RCLCPP_INFO(this->get_logger(), "MCLNode::InitOdom!");
        return;  
      }

      Eigen::Vector4d currPose = OdomMsg2Pose(odom_msg);
      Eigen::Vector4d delta = currPose - o_prevPose; 

      if(((delta.norm() > o_triggerDist) || (fabs(delta(3)) > o_triggerAngle)))
      {

          Eigen::Vector4d u = o_mcl->Backward(o_prevPose, currPose);
          o_mtx->lock();
          o_mcl->Predict(u, o_odomNoise);
          std::vector<Particle> particles = o_mcl->Particles();  
          publishParticles(particles, odom_msg->header.stamp);
          o_mtx->unlock(); 

          o_prevPose = currPose;

      }
    }
 
    void observationCallback(const signloc_msgs::msg::Sign::SharedPtr sign_msg)
    {
        RCLCPP_INFO(this->get_logger(), "MCLNode::observationCallback");

        if (!o_initGPS) return;
        std::unordered_map<std::string, int > actionDict = {{"RIGHT", 0}, {"STRAIGHT-RIGHT",1}, {"STRAIGHT", 2}, {"STRAIGHT-LEFT", 3}, {"LEFT", 4}, {"BACK-LEFT", 5}, {"BACK", 6}, {"BACK-RIGHT", 7}, {"LOCATIONAL", 8}, {"UP", 9}, {"DOWN", 10}};
        std::unordered_map<std::string, std::vector<std::tuple<int, double>>> minigraph;
        std::vector<signloc_msgs::msg::SignElement> elements = sign_msg->elements;

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

            RCLCPP_INFO(this->get_logger(), "%s", std::string("Place label " + place).c_str());
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
                
                //ROS_INFO_STREAM("Action " + std::to_string(action) + " Prob " + std::to_string(prob)); 

                RCLCPP_INFO(this->get_logger(), "Action %d Prob %f", action, prob);

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
          //ROS_INFO_STREAM( i <<"th best particle is (" << particles[i].ID() << "," << particles[i].Pose()(3) << ") , with weight " << particles[i].Weight());

          RCLCPP_INFO(this->get_logger(), "%dth best particle is (%d, %f) with weight %f", i, particles[i].ID(), particles[i].Pose()(3), particles[i].Weight());
        }
        o_mcl->Resampling();

        publishPrediction(pose, sign_msg->header.stamp);
        publishParticles(particles, sign_msg->header.stamp); 
        o_armStatus = "none";
        RCLCPP_INFO(this->get_logger(), "MCLNode::arm status %s", o_armStatus.c_str());  


        o_mtx->unlock();  
        o_initObservation = true;
        RCLCPP_INFO(this->get_logger(), "MCLNode::Sign!");
    }
 
    void publishPrediction(Eigen::Vector4d pose, const rclcpp::Time& stamp)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = o_mapTopic;
      marker.header.stamp = stamp;
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
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
      marker.lifetime = rclcpp::Duration::from_seconds(500);
      o_markerPub->publish(marker); 
    }





     void publishParticles(const std::vector<Particle>& particles, const rclcpp::Time& stamp)
     {
        geometry_msgs::msg::PoseArray  posearray;
        posearray.header.stamp = stamp;  
        posearray.header.frame_id = o_mapTopic;
        posearray.poses = std::vector<geometry_msgs::msg::Pose>(particles.size());

        for (int i = 0; i < particles.size(); ++i)
        {
            geometry_msgs::msg::Pose p;
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
            posearray.poses[i] = p;
        }

        o_particlePub->publish(posearray);
     }



  private:   
 
    

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr o_posePub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr o_particlePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr o_markerPub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr o_odomSub;
    rclcpp::Subscription<signloc_msgs::msg::Sign>::SharedPtr o_signSub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr o_armSub;
    rclcpp::Subscription<signloc_msgs::msg::UTM>::SharedPtr o_gpsSub;




    
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
    std::string o_navGraphPath;
    bool o_initGPS = false;

   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MCLNode>());
  rclcpp::shutdown();
  return 0;
}
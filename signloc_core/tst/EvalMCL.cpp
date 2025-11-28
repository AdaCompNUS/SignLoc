/**
# ##############################################################################
#  Copyright (c) 2024- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: EvalMCL.cpp             		                           		       #
# ##############################################################################
**/


#include <iostream>
#include <math.h>
#include <string>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <string>
#include "NavGraph.h"
#include "MCL.h"
#include "NavOccMap.h"
#include "JSONLoader.h"
#include "ActionMotionModel.h"
#include "Utils.h"


std::vector<std::string> S7 = {
  "/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/21/frame_0011.jpg",
"/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/16/frame_0002.jpg",
 "/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM3_L1_new/frame_0069.jpg",
 "/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM3_L1_new/frame_0323.jpg",
  "/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM2_L1_new/frame_0167.jpg"

};


std::vector<std::string> S8 = {
   "/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM3_L1_new/frame_0005.jpg", 
   "/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM2_L1_new/frame_0097.jpg",
"/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM2_L1_new/frame_0439.jpg",
"/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/15/frame_0015.jpg",
};

std::vector<std::string> S9 = {
  "/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM2_L1_new/frame_0097.jpg",
 "/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM2_L1_new/frame_0195.jpg",
"/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/20/frame_0017.jpg",
"/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/21/frame_0011.jpg",
 "/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/19/frame_0008.jpg"
};




double circularDistance(double rad1, double rad2)
{
    double diff = abs(WrapTo2Pi(rad1) - WrapTo2Pi(rad2));
    return std::min(diff, 2 * M_PI - diff);
}

void EvalPrediction( std::vector<State> GTs, std::vector<State> preds)
{
    int cnt = 0;
    bool fail = false;
    for(size_t i = 0; i < preds.size(); ++i)
    {
        if (std::get<0>(GTs[i]) == std::get<0>(preds[i]))
        {
            double diff = circularDistance(std::get<1>(GTs[i]), std::get<1>(preds[i]));
            if (diff < M_PI * 0.25) ++cnt;
            else
            {
                if (cnt > 0)
                {
                    std::cout << "localization failed!" << std::endl;
                    fail = true;
                    break;
                }
            }
        }
    }
    if (!fail)
    {
        std::cout << "localized on sign " << preds.size() - cnt << "/" << preds.size()  << std::endl;
    }
}


int main(int argc, char **argv) {
   
   	std::string NavGraphPath = "/GraphLoc/data_backup/NUS_navgraph.pickle";
    Eigen::Vector2d origin(363784.0139568621, 143122.7189409806);
    JSONLoader jl = JSONLoader("/GraphLoc/ros_ws/src/graphloc_ros/core/tst/", "/GraphLoc/Test/NUS-response.json");

    std::vector<std::string> S = S7;
	MCL mcl = MCL(NavGraphPath, -1, origin, false); 

    std::vector<State> GTs;
    std::vector<State> preds;

    for(size_t i = 0; i < S.size(); ++i)
    {
        std::string framePath = S[i];
        MiniGraph minigraph = jl.GetPrediction(framePath);
        Eigen::Vector4d gt1 = jl.GetGT(framePath);
    
        mcl.Correct(minigraph);

        std::vector<Particle> particles = mcl.Particles(); 
        std::sort(particles.begin(), particles.end(), [](const Particle& p1, const Particle& p2) {
        return p1.Weight() > p2.Weight(); 
        });

        int nodeID = std::get<0>(mcl.Graph()->FindClosestNode(gt1));

        std::cout << "GT: " << nodeID << ", " << WrapTo2Pi(gt1(3))<< std::endl;
        for (int i = 0; i < 5; ++i)
        {
            std::cout << i <<"th best particle is (" << particles[i].ID() << "," << particles[i].Pose()(3) << ") , with weight " << particles[i].Weight() << std::endl;
        }

        State predState = std::make_tuple(particles[0].ID(), particles[0].Pose()(3));
        preds.push_back(predState);
        State gtState = std::make_tuple(nodeID, gt1(3));
        GTs.push_back(gtState);

        if (i + 1 < S.size())
        {
            std::string nextFramePath = S[i + 1];
            Eigen::Vector4d gt2 = jl.GetGT(nextFramePath);

            mcl.NormalizeWeights();
            mcl.PredictAction(gt1, gt2);
        }
    }

    EvalPrediction( GTs, preds);

}
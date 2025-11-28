/**
# ##############################################################################
#  Copyright (c) 2024- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: TestGraph.cpp             		                           		       #
# ##############################################################################
**/


#include "gtest/gtest.h"
#include <opencv2/opencv.hpp>
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

std::string dataPath = PROJECT_TEST_DATA_DIR;



// TEST(TestNavGraph, test1) {

// 	std::string modulePath = "/GraphLoc/ros_ws/src/graphloc_ros/core/src/";
// 	std::string NavGraphPath = "/GraphLoc/data/NUS_navgraph.pickle";
    
// 	NavGraph ng = NavGraph(modulePath, NavGraphPath);  

// }


// TEST(TestNavOccMap, test1) {

// 	std::string modulePath = "/GraphLoc/ros_ws/src/graphloc_ros/core/src/";
// 	std::string NavGraphPath = "/GraphLoc/data/NUS_navgraph.pickle";
// 	NavGraph ng = NavGraph(modulePath, NavGraphPath);  

// 	std::vector<std::string> edgeTypes = {"action"};
// 	std::unordered_map<int, std::unordered_map<int, std::shared_ptr<NavGraphEdge>>> actionEdges = ng.FindEdgesOfType(edgeTypes);

// 	std::vector<std::vector<Eigen::Vector3d>> edgeLines;
// 	for (auto item : actionEdges) 
// 	{
// 		int u = item.first;
// 		std::unordered_map<int, std::shared_ptr<NavGraphEdge>> uEdges = item.second;
// 		Eigen::Vector3d uPos = ng.Node(u)->Pos();

// 		for (auto item2 : uEdges) 
// 		{
// 			int v = item2.first;
// 			Eigen::Vector3d vPos = ng.Node(v)->Pos();

// 			std::vector<Eigen::Vector3d> uvLine = {uPos, vPos};
// 			edgeLines.push_back(uvLine);

// 			//std::cout << u << ", " << v << std::endl;
// 			//std::cout << uPos << ", " << vPos << std::endl;
// 		}
// 	}


// 	double resolution = 0.5;
// 	int radius = 400;
// 	Eigen::Vector2d origin(363784.0139568621, 143122.7189409806);

// 	NavOccMap(edgeLines, origin, resolution, radius);
// }


// TEST(TestMCL, test1) {

// 	std::string NavGraphPath = "/GraphLoc/data/NUS_navgraph.pickle";
    
// 	MCL mcl = MCL(NavGraphPath, 1, Eigen::Vector2d()); 

// 	std::unordered_map<std::string, std::vector<std::tuple<int, double>>> minigraph;
// 	std::vector<std::tuple<int, double>> actionList;
// 	actionList.push_back(std::make_tuple(2, 1.0));
// 	minigraph[std::string("Terrace")] = actionList;

// 	mcl.ParseMiniGraph(minigraph);
// }


// TEST(TestMCL, test2) {

// 	std::string NavGraphPath = "/GraphLoc/data/NUS_navgraph.pickle";
//     Eigen::Vector2d origin(363784.0139568621, 143122.7189409806);

// 	MCL mcl = MCL(NavGraphPath, -1, origin); 


// 	Eigen::Vector4d p1(1.0, -2.2, 0.03, 0.45);
// 	Eigen::Vector4d p2(1.1, -2.17, 0.04, 0.51);
// 	Eigen::Vector4d command = mcl.Backward(p1, p2);
// 	Eigen::Vector4d noise(0.1, 0.1, 0.01, 0.1);


// 	auto start = std::chrono::high_resolution_clock::now();
// 	mcl.Predict(command, noise);
// 	auto end = std::chrono::high_resolution_clock::now();
//     auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//     std::cout << "Function took " << duration.count() << " microseconds." << std::endl;
// }

TEST(TestMCL, test3) {

    std::string NavGraphPath = "/GraphLoc/data/NUS_navgraph.pickle";
    Eigen::Vector2d origin(363784.0139568621, 143122.7189409806);

    MCL mcl = MCL(NavGraphPath, -1, origin); 

    std::unordered_map<std::string, std::vector<std::tuple<int, double>>> minigraph;
    std::vector<std::tuple<int, double>> actionList;
    actionList.push_back(std::make_tuple(0, 1.0));
    minigraph[std::string("COM4")] = actionList;

    std::vector<std::tuple<int, double>> actionList2;
    actionList2.push_back(std::make_tuple(0, 1.0));
    minigraph[std::string("i4.0")] = actionList2;

    std::vector<std::tuple<int, double>> actionList3;
    actionList3.push_back(std::make_tuple(4, 1.0));
    minigraph[std::string("COM3")] = actionList3;

    auto start = std::chrono::high_resolution_clock::now();
    mcl.Correct(minigraph);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Function Correct took " << duration.count() << " microseconds." << std::endl;

    std::vector<Particle> particles = mcl.Particles(); 
    std::sort(particles.begin(), particles.end(), [](const Particle& p1, const Particle& p2) {
        return p1.Weight() > p2.Weight();  // Returns true if 'a' should come before 'b' (for descending)
    });

    // for (int i = 0; i < 20; ++i)
    // {
    //      std::cout << i <<"th best particle is (" << particles[i].ID() << "," << particles[i].Pose()(3) << ") , with weight " << particles[i].Weight() << std::endl;
    // }

    ASSERT_EQ(particles[0].ID(), 498);

    return;

    start = std::chrono::high_resolution_clock::now();
    mcl.Resampling();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Function Resampling took " << duration.count() << " microseconds." << std::endl;

    std::vector<Particle> particles2 = mcl.Particles(); 

   std::unordered_map<int, int> frequency_map; 
   for (int i = 0; i < particles2.size(); ++i)
    {
        frequency_map[particles2[i].ID()]++;
    }

    auto max_it = std::max_element(frequency_map.begin(), frequency_map.end(),
                                   [](const auto& p1, const auto& p2) {
                                       return p1.second < p2.second;
                                   });
   //  //std::cout << "The element " <<  max_it->first << " appears " << max_it->second << " times." << std::endl;
   //  // for(auto item: frequency_map)
   //  // {
   //  //  std::cout << "The element " << item.first << " appears " << item.second << " times." << std::endl;
   //  // }
   ASSERT_EQ(max_it->first, 498);

}

TEST(TestMCL, test4) {

	std::string NavGraphPath = "/GraphLoc/data/NUS_navgraph.pickle";
    Eigen::Vector2d origin(363784.0139568621, 143122.7189409806);

	MCL mcl = MCL(NavGraphPath, -1, origin); 

	std::unordered_map<std::string, std::vector<std::tuple<int, double>>> minigraph;

	// std::vector<std::tuple<int, double>> actionList;
	// actionList.push_back(std::make_tuple(2, 1.0));
	// minigraph[std::string("COM1&COM2")] = actionList;

	// std::vector<std::tuple<int, double>> actionList2;
	// actionList2.push_back(std::make_tuple(2, 1.0));
	// minigraph[std::string("SECURITYPOST@COM2")] = actionList2;

	// std::vector<std::tuple<int, double>> actionList3;
	// actionList3.push_back(std::make_tuple(2, 1.0));
	// minigraph[std::string("BUSSTOP")] = actionList3;

    // std::vector<std::tuple<int, double>> actionList4;
    // actionList4.push_back(std::make_tuple(2, 1.0));
    // minigraph[std::string("CARPARK13")] = actionList4;

    // std::vector<std::tuple<int, double>> actionList5;
    // actionList5.push_back(std::make_tuple(4, 1.0));
    // minigraph[std::string("BIZ2")] = actionList5;


    std::vector<std::tuple<int, double>> actionList;
    actionList.push_back(std::make_tuple(2, 1.0));
    minigraph[std::string("COM3")] = actionList;

    std::vector<std::tuple<int, double>> actionList2;
    actionList2.push_back(std::make_tuple(2, 1.0));
    minigraph[std::string("THETERRACE")] = actionList2;

    std::vector<std::tuple<int, double>> actionList3; 
    actionList3.push_back(std::make_tuple(2, 1.0));
    minigraph[std::string("INNOVATION4.0")] = actionList3;

    std::vector<std::tuple<int, double>> actionList4;
    actionList4.push_back(std::make_tuple(0, 1.0));
    minigraph[std::string("BIZ2")] = actionList4;

    // std::vector<std::tuple<int, double>> actionList5;
    // actionList5.push_back(std::make_tuple(4, 1.0));
    // minigraph[std::string("TOILETS")] = actionList5;

    std::vector<std::tuple<int, double>> actionList6;
    actionList6.push_back(std::make_tuple(0, 1.0));
    minigraph[std::string("LIFTS")] = actionList6;


//     // std::vector<std::tuple<int, double>> actionList;
//     // actionList.push_back(std::make_tuple(6, 1.0));
//     // minigraph[std::string("THETERRACE")] = actionList;

//     // std::vector<std::tuple<int, double>> actionList2;
//     // actionList2.push_back(std::make_tuple(2, 1.0));
//     // minigraph[std::string("SEMINARROOM11(ALC)")] = actionList2;

//     // std::vector<std::tuple<int, double>> actionList3;
//     // actionList3.push_back(std::make_tuple(2, 1.0));
//     // minigraph[std::string("MULTIPURPOSEHALLS1-3")] = actionList3;

	auto start = std::chrono::high_resolution_clock::now();
	mcl.Correct(minigraph);
	auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Function Correct took " << duration.count() << " microseconds." << std::endl;

    std::vector<Particle> particles = mcl.Particles(); 
    std::sort(particles.begin(), particles.end(), [](const Particle& p1, const Particle& p2) {
        return p1.Weight() > p2.Weight();  // Returns true if 'a' should come before 'b' (for descending)
    });

    for (int i = 0; i < 20; ++i)
    {
    	 std::cout << i <<"th best particle is (" << particles[i].ID() << "," << particles[i].Pose()(3) << ") , with weight " << particles[i].Weight() << std::endl;
    }

    ASSERT_EQ(particles[0].ID(), 216);

    start = std::chrono::high_resolution_clock::now();
    mcl.Resampling();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Function Resampling took " << duration.count() << " microseconds." << std::endl;

    std::vector<Particle> particles2 = mcl.Particles(); 

   std::unordered_map<int, int> frequency_map; 
   for (int i = 0; i < particles2.size(); ++i)
    {
    	frequency_map[particles2[i].ID()]++;
    }

    auto max_it = std::max_element(frequency_map.begin(), frequency_map.end(),
                                   [](const auto& p1, const auto& p2) {
                                       return p1.second < p2.second;
                                   });
   
    ASSERT_EQ(max_it->first, 216);

}


TEST(TestMCL, test5) {


    JSONLoader jl = JSONLoader("/GraphLoc/ros_ws/src/graphloc_ros/core/tst/", "/GraphLoc/Test/COM2_COM3_L1-response.json");
    MiniGraph minigraph = jl.GetPrediction("/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM3_L1_new/frame_0069.jpg");
    

    std::string NavGraphPath = "/GraphLoc/data/NUS_navgraph.pickle";
    Eigen::Vector2d origin(363784.0139568621, 143122.7189409806);

    MCL mcl = MCL(NavGraphPath, -1, origin); 

    // std::unordered_map<std::string, std::vector<std::tuple<int, double>>> minigraph;
    //  std::vector<std::tuple<int, double>> actionList;
    // actionList.push_back(std::make_tuple(4, 1.0));
    // minigraph[std::string("MULTIPURPOSEHALLS1-3")] = actionList;

    // std::vector<std::tuple<int, double>> actionList2;
    // actionList2.push_back(std::make_tuple(2, 1.0));
    // minigraph[std::string("SEMINARROOM15(ALC)")] = actionList2;

    // std::vector<std::tuple<int, double>> actionList3; 
    // actionList3.push_back(std::make_tuple(2, 1.0));
    // minigraph[std::string("SEMINARROOM13(ALC)")] = actionList3;

    // std::vector<std::tuple<int, double>> actionList4; 
    // actionList4.push_back(std::make_tuple(2, 1.0));
    // minigraph[std::string("SEMINARROOM14")] = actionList4;


    auto start = std::chrono::high_resolution_clock::now();
    mcl.Correct(minigraph);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Function Correct took " << duration.count() << " microseconds." << std::endl;

    std::vector<Particle> particles = mcl.Particles(); 
    std::sort(particles.begin(), particles.end(), [](const Particle& p1, const Particle& p2) {
        return p1.Weight() > p2.Weight();  // Returns true if 'a' should come before 'b' (for descending)
    });

    for (int i = 0; i < 20; ++i)
    {
         std::cout << i <<"th best particle is (" << particles[i].ID() << "," << particles[i].Pose()(3) << ") , with weight " << particles[i].Weight() << std::endl;
    }

    ASSERT_EQ(particles[0].ID(), 118);

    start = std::chrono::high_resolution_clock::now();
    mcl.Resampling();
    end = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Function Resampling took " << duration.count() << " microseconds." << std::endl;

    std::vector<Particle> particles2 = mcl.Particles(); 

   std::unordered_map<int, int> frequency_map; 
   for (int i = 0; i < particles2.size(); ++i)
    {
        frequency_map[particles2[i].ID()]++;
    }

      auto max_it = std::max_element(frequency_map.begin(), frequency_map.end(),
                                   [](const auto& p1, const auto& p2) {
                                       return p1.second < p2.second;
                                   });
   
     ASSERT_EQ(max_it->first, 118);

}




TEST(TestActionMotionModel, test1) {

    JSONLoader jl = JSONLoader("/GraphLoc/ros_ws/src/graphloc_ros/core/tst/", "/GraphLoc/Test/COM2_COM3_L1-response.json");
 
    Eigen::Vector4d gt1 = jl.GetGT("/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM3_L1_new/frame_0005.jpg");
    Eigen::Vector4d gt2 = jl.GetGT("/home/ayush/SignLoc/sign-gpt/ros-wrkspc/src/smpl_pkg/datasets/sign_recog_frames/temp/all_frames/COM2_L1_new/frame_0097.jpg");


    std::string modulePath = "/GraphLoc/ros_ws/src/graphloc_ros/core/src/";
	std::string NavGraphPath = "/GraphLoc/data/NUS_navgraph.pickle";

    std::shared_ptr<NavGraph> navGraph = std::make_shared<NavGraph>(modulePath, NavGraphPath);
    int actionNum = 8;
    std::shared_ptr<ActionMotionModel>  mm = std::make_shared<ActionMotionModel>(navGraph, actionNum);

    Control ctrl = mm->Backward(gt1, gt2);
    std::vector<int> actions = std::get<0>(ctrl);
    double orientationChange = std::get<1>(ctrl);

    // for(size_t i = 0; i < actions.size(); ++i)
    // {
    //     std::cout << actions[i] << ",";
    // }
    // std::cout << std::endl;
    // std::cout << orientationChange << std::endl;


    ASSERT_EQ(actions[0], 6);
    ASSERT_EQ(actions[1], 3);
    ASSERT_NEAR(orientationChange, -2.356194490192345, 0.01);

    int nodeID = std::get<0>(navGraph->FindClosestNode(gt1));

    State state = std::make_tuple(nodeID, gt1(3));
    State nextState = mm->Forward(state, ctrl);

    // std::cout << std::get<0>(nextState) << ", " <<  std::get<1>(nextState) << std::endl;
    // std::cout <<  WrapTo2Pi(gt2(3)) << std::endl;

    ASSERT_EQ(std::get<0>(nextState), 216);
    ASSERT_NEAR(std::get<1>(nextState), WrapTo2Pi(gt2(3)), 0.01);


    // [156, 100, 99, 137, 135, 104, 106, 153, 107, 105, 216]
    //[6, 3, 3, 2, 2, 3, 2, 2, 1, 1] -2.356194490192345
    
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
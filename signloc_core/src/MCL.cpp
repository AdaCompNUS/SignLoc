/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MCL.cpp                                                               #
# ##############################################################################
**/


#include "MCL.h"
#include <numeric>
#include <functional>  
#include <iostream>
#include <fstream>
#include "Utils.h"
#include <algorithm>
#include <iterator>
#include <boost/filesystem.hpp>
#include <exception>
#include <cmath>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/embed.h>
#include <pybind11/stl.h>
#include <cmath>
#include <math.h>


namespace py = pybind11;
using namespace py::literals;

MCL::MCL(const std::string& NavGraphPath, int n_particles, Eigen::Vector2d center, bool ros, double sigma)
{

	o_gpsCenter = center;
	o_motionModel = std::shared_ptr<FSR>();
	o_numParticles = n_particles;

	o_ros = ros;
	o_sigma = sigma;
	

	o_navGraph = std::make_shared<NavGraph>("/ros_ws/src/signloc_ros/signloc_core/src/", NavGraphPath);
	this->createOccupancyGrid();

	o_actionMotionModel = std::make_shared<ActionMotionModel>(o_navGraph, o_actionNum);
	
	std::vector<std::string> nodeTypes = {"room", "floor", "building"};
	std::unordered_map<int, std::shared_ptr<NavGraphNode>> placeNodes = o_navGraph->FindNodesOfType(nodeTypes);
	for (auto item : placeNodes) 
	{
		int nodeID = item.first;
        std::shared_ptr<NavGraphNode> ngn = item.second;

        if (ngn->o_name.length() > 1)
        {
        	o_placeNodeNames.push_back(ngn->Name());
        	o_placeNodesIDs.push_back(nodeID);
        }
	}

	try {
		py::initialize_interpreter();
	}
	catch (...) {
	  std::cout << "Interpreter already initialized!" << std::endl;
	}
	py::module sys = py::module::import("sys");
	py::module np = py::module::import("numpy");

	py::list path = sys.attr("path");
	path.attr("append")("/ros_ws/src/signloc_ros/signloc_core/src/");

	o_fuzzyWrapper = py::module::import("fuzzy_wrap");
	o_matchLabels = o_fuzzyWrapper.attr("matchLabels");

	this->InitParticles();
}


void MCL::createOccupancyGrid()
{
	std::vector<std::string> edgeTypes = {"action"};
	std::unordered_map<int, std::unordered_map<int, std::shared_ptr<NavGraphEdge>>> actionEdges = o_navGraph->FindEdgesOfType(edgeTypes);
	std::vector<std::vector<Eigen::Vector3d>> edgeLines;
	for (auto item : actionEdges) 
	{
		int u = item.first;
		std::unordered_map<int, std::shared_ptr<NavGraphEdge>> uEdges = item.second;
		Eigen::Vector3d uPos = o_navGraph->Node(u)->Pos();

		for (auto item2 : uEdges) 
		{
			int v = item2.first;
			Eigen::Vector3d vPos = o_navGraph->Node(v)->Pos();

			std::vector<Eigen::Vector3d> uvLine = {uPos, vPos};
			edgeLines.push_back(uvLine);
		}
	}


	double resolution = 0.5;
	int radius = 400;
	o_occMap = std::make_shared<NavOccMap>(edgeLines, o_gpsCenter, resolution, radius);
}


GraphLocData MCL::ParseMiniGraph(const MiniGraph& minigraph)
{

	std::vector<std::vector<std::tuple<std::string, int, double>>>placeNodeDistList;
	std::vector<std::vector<std::tuple<int, double>>> actionDistList;


	for (auto item : minigraph) 
	{
		std::vector<std::tuple<std::string, int, double>> matchesIDandProb;

		std::string place = item.first;
        std::vector<std::tuple<int, double>> actionDist = item.second;
		py::list matchList = o_matchLabels(place, o_placeNodeNames);

		double totalScore = 0.0;

		for (size_t i = 0; i < matchList.size(); ++i)
		{
			py::tuple m = matchList[i];
			std::tuple<std::string, double> match = m.cast<std::tuple<std::string, double>>();
			std::string place = std::get<0>(match);
			//std::cout << std::get<0>(match) << std::endl;

			auto it = std::find(o_placeNodeNames.begin(), o_placeNodeNames.end(), place);
			int index = std::distance(o_placeNodeNames.begin(), it);
			int nodeID = o_placeNodesIDs[index];

			double score = std::exp(-(100 - std::get<1>(match)));
			totalScore += score;

			std::tuple<std::string, int, double> matchAndProb = std::make_tuple(place, nodeID, score);
			matchesIDandProb.push_back(matchAndProb);
		}
		// normalize the scores
		for (size_t i = 0; i < matchList.size(); ++i)
		{
			double score = std::get<2>(matchesIDandProb[i]) / totalScore;
			std::string place = std::get<0>(matchesIDandProb[i]);
			int nodeID = std::get<1>(matchesIDandProb[i]);
			std::tuple<std::string, int, double> matchAndProb = std::make_tuple(place, nodeID, score);
			matchesIDandProb[i] = matchAndProb;
		}


		actionDistList.push_back(actionDist);
		placeNodeDistList.push_back(matchesIDandProb);
	}

	GraphLocData data = std::make_tuple(placeNodeDistList, actionDistList);

	return data;

}



void MCL::Correct(const MiniGraph& minigraph)
{
	GraphLocData data = this->ParseMiniGraph(minigraph);

	std::vector<std::vector<std::tuple<std::string, int, double>>> placeNodeDistList = std::get<0>(data);
	std::vector<std::vector<std::tuple<int, double>>> actionDistList = std::get<1>(data);

	int labelNum = placeNodeDistList.size();

	//#pragma omp parallel for 
	for(int i = 0; i < o_numParticles; ++i)
	{
		int currNodeID = o_particles[i].ID();
		//double currOrientation = o_particles[i].Pose()(3); 
		//if (o_ros) currOrientation = o_particles[i].Pose()(3)- 0.5 * M_PI;
		double currOrientation = o_particles[i].Pose()(3)- 0.5 * M_PI;


		double totalW = 1.0;


		for (size_t i = 0; i < placeNodeDistList.size(); ++i)
		{
			std::vector<std::tuple<std::string, int, double>> placeNodeDist = placeNodeDistList[i];
			std::vector<std::tuple<int, double>> actionDist = actionDistList[i];

			double locW = 0.0;

			for (size_t l = 0; l < placeNodeDist.size(); ++l)
			{
				std::tuple<std::string, int, double> pnd = placeNodeDist[l];
				std::string placeNode = std::get<0>(pnd);
				int placeNodeID = std::get<1>(pnd);
				double placeProb = std::get<2>(pnd);

				//std::cout << placeNode << " " << placeNodeID << " " << placeProb << std::endl;
				if (placeProb < 0.3) break;


				double actionW = 0.0;

				Eigen::Vector3d currPos = o_navGraph->Node(currNodeID)->Pos();

				std::string currType = o_navGraph->Node(currNodeID)->Type();

				if (o_navGraph->Node(placeNodeID)->Type() == "room")
				{
					std::vector<std::string> nodeTypes = {"portal"};
					std::unordered_map<int, std::shared_ptr<NavGraphNode>> portalNodes = o_navGraph->FindConnectedNodesOfType(placeNodeID, nodeTypes);
					if (portalNodes.size())
					{
						double minDist = DBL_MAX;
						int minDistID = -1;
						for (auto item : portalNodes)
						{
							int v = item.first;
							Eigen::Vector3d neighborPos = o_navGraph->Node(v)->Pos();
							double dist =  (currPos - neighborPos).norm();
							if (dist < minDist)
							{
								minDist = dist;
								minDistID = v;
							}
						}
						placeNodeID = minDistID; // now trying to reach the nearest door instead
					}
				}
		

				std::vector<int> path = o_navGraph->Path(currNodeID, placeNodeID);


				if (path.size() > 1)
				{
	                int nextNodeId = path[1];
	            	std::string edgeType = o_navGraph->Edge(currNodeID, nextNodeId)->Type();
	                if (edgeType == "action")
	                {
	                	Eigen::Vector3d nextPos = o_navGraph->Node(nextNodeId)->Pos();
	                	Eigen::Vector3d nextPossibleDirc = (nextPos - currPos).normalized();

	                	for (size_t a = 0; a < actionDist.size(); ++a)
	                	{
	                		std::tuple<int, double> ap = actionDist[a];
	                		int action = std::get<0>(ap);
	                		double actionProb = std::get<1>(ap);

	                		// Computing the nextOrientation, upon taking the action on the sign
	                		double nextOrientation = WrapTo2Pi(currOrientation + action2rad(action, o_actionNum));
	                		// we convert it back to a unit vector, this should be our new approximate heading after taking this action
	                		Eigen::Vector2d nextPredictedDirc = rad2Vec(nextOrientation);
	                		double angleDiff = acos(nextPossibleDirc.head(2).dot(nextPredictedDirc));
	                		actionW += actionProb * std::exp(- std::pow((angleDiff/ o_sigma), 2.0));
	                	}                    
	                }
	                else actionW += std::exp(- std::pow((0.5 * M_PI/ o_sigma), 2.0));
	            }
	            else actionW += std::exp(- std::pow((0.5 * M_PI/ o_sigma), 2.0));

	            locW += placeProb * actionW;
			}
			totalW *= locW;
		}

		//totalW = std::pow((totalW), 1.0/double(labelNum));

		o_particles[i].Weight(totalW * o_particles[i].Weight());
		// if (o_particles[i].ID() == 498)
		// {
		// 	std::cout.precision(12);

		// 	std::cout << totalW << std::endl;
		// }
	}
}


void MCL::InitParticles()
{
	std::vector<std::string> nodeTypes = {"terminal", "intersection"};
	o_traverseNodes = o_navGraph->FindNodesOfType(nodeTypes);
	o_nodeNum = o_traverseNodes.size();

	for (auto item : o_traverseNodes)
	{
		o_traverseNodeIDs.push_back(item.first);
	}

	if (o_numParticles == -1)
	{
		o_numParticles = o_nodeNum * o_orientationNum;

		//o_particles = std::vector<Particle>(o_numParticles);
		//for (int n = 0; n < o_nodeNum; ++n)
		for (auto item : o_traverseNodes)
		{
			for (int o = 0; o < o_orientationNum; ++o)
			{
				int nodeID = item.first;
				std::shared_ptr<NavGraphNode> node = item.second;

				double orientation = -0.5 * M_PI + o * 2 * M_PI / o_orientationNum;
				Eigen::Vector3d position = node->Pos();
				Eigen::Vector4d pose(position(0), position(1), position(2), orientation);
				Particle p(pose, nodeID, 1.0);
				o_particles.push_back(p);
				//o_particles[n * o_orientationNum + o] = Particle()
			}
		}
		//std::cout << "particle size is " << o_particles.size() << " and it should be " << o_numParticles << std::endl;
	}
}


Particle MCL::initParticleUniform()
{
	int idx = rand() % o_nodeNum;
	int nodeID = o_traverseNodeIDs[idx];
	Eigen::Vector3d position = o_traverseNodes[nodeID]->Pos();
	double orientation = -0.5 * M_PI + (rand() % o_orientationNum) * 2 * M_PI / o_orientationNum;
	Eigen::Vector4d pose(position(0), position(1), position(2), orientation);
	Particle  p(pose, nodeID, 1.0);

	return p;
}

Particle MCL::initParticleFromNodeID(int nodeID)
{
	double chance = drand48();
	Particle p;

	if (chance < 0.95)
	{
		std::vector<int> sameNodeParticleIDs;
		for (int i = 0; i < o_numParticles; ++i)
		{
			if (o_particles[i].ID() == nodeID)
			{
				sameNodeParticleIDs.push_back(i);
			}
		}

		if (sameNodeParticleIDs.size() > 1)
		{
			int idx = rand() % sameNodeParticleIDs.size();
			p = o_particles[sameNodeParticleIDs[idx]];
		}	
		else
		{
			int idx = rand() % o_numParticles;
			p = o_particles[idx];
		}
	}
	else
	{
		int idx = rand() % o_numParticles;
		p = o_particles[idx];

		//std::cout << "whoop " << p.ID() << std::endl;
	}
	return p;
}


void MCL::Predict(Eigen::Vector4d& u, const Eigen::Vector4d& noise)
{
	//#pragma omp parallel for 
	for(int i = 0; i < o_numParticles; ++i)
	{
		Eigen::Vector4d pose = o_motionModel->SampleMotion(o_particles[i].Pose(), u, noise);
		o_particles[i].Pose(pose);

		int nodeID = o_particles[i].ID();
		
		//particle pruning - if particle is outside the map, we replace it
		while (!o_occMap->IsValid(o_particles[i].Pose().head(2)))
		{
			//Particle p = this->initParticleUniform();
			Particle p = this->initParticleFromNodeID(nodeID);
			o_particles[i] = p;
			nodeID = p.ID();
		}

		Eigen::Vector4d currPose = o_particles[i].Pose();
		std::tuple<int, double> result = o_navGraph->FindClosestNeighborNode(nodeID, currPose);
		int newNodeID = std::get<0>(result);
		o_particles[i].ID(newNodeID);
	}
}

void MCL::PredictAction(Eigen::Vector4d& p1, const Eigen::Vector4d& p2)
{

	Control ctrl = o_actionMotionModel->Backward(p1, p2);
	//#pragma omp parallel for 
	for(int i = 0; i < o_numParticles; ++i)
	{
		//std::cout << i << "/" << o_numParticles << std::endl;
		int nodeID = o_particles[i].ID();
		double currOrientation = o_particles[i].Pose()(3);

		//std::cout << nodeID << ", " << currOrientation << std::endl;

		State state = std::make_tuple(nodeID, currOrientation);
    	State nextState = o_actionMotionModel->Forward(state, ctrl);
		int nextNodeID = std::get<0>(nextState);
		double nextOrientation = std::get<1>(nextState);

		// std::cout << nextNodeID << ", " << nextOrientation << std::endl;
		// std::cout << std::endl;

		Eigen::Vector3d position = o_navGraph->Node(nextNodeID)->Pos();
		Eigen::Vector4d pose(position(0), position(1), position(2), nextOrientation);

		o_particles[i].ID(nextNodeID);
		o_particles[i].Pose(pose);
	}

	//std::cout << "done" <<std::endl;
}

void MCL::NormalizeWeights()
{
	std::vector<double> transWeights(o_numParticles);
	std::vector<Particle> new_particles(o_numParticles);

	double w = 0;
	for(long unsigned int i = 0; i < o_numParticles; ++i)
	{
		w += o_particles[i].Weight();
	}

	for(long unsigned int i = 0; i < o_numParticles; ++i)
	{
		o_particles[i].Weight(o_particles[i].Weight() / w);
	}
}


void MCL::Resampling()
{
	std::vector<double> transWeights(o_numParticles);
	std::vector<Particle> new_particles(o_numParticles);

	double w = 0;
	for(long unsigned int i = 0; i < o_numParticles; ++i)
	{
		w += o_particles[i].Weight();
	}

	for(long unsigned int i = 0; i < o_numParticles; ++i)
	{
		o_particles[i].Weight(o_particles[i].Weight() / w);
	}


	std::transform(o_particles.begin(), o_particles.end(), transWeights.begin(), [](Particle &p){ return p.Weight() * p.Weight(); });
	double sumWeights = std::accumulate(transWeights.begin(), transWeights.end(), 0.0);

	double r1 = 0;
    double r2 = 12;
	

	double effN = 1.0 / sumWeights;

	if (effN < o_efficiencyCoeff * o_numParticles)
	{
		//std::cout << "resampling" << std::endl;
		//double unitW = 1.0 / o_numParticles;
		double r = drand48() * 1.0 / o_numParticles;
		double acc = o_particles[0].Weight();
		int i = 0;

		for(int j = 0; j < o_numParticles; ++j)
		{
			double U = r + j * 1.0 / o_numParticles;
			while((U > acc) && (i < o_numParticles - 1))
			{
				++i;
				acc += o_particles[i].Weight();
				//std::cout << i << std::endl;
			}
			//new_particles[j] = o_particles[i];
			//new_particles[j].Weight(unitW);

			int currNodeID = o_particles[i].ID(); 
			Eigen::Vector3d nodePos = o_navGraph->Node(currNodeID)->Pos(); 
			double orientation = o_particles[i].Pose()(3);

			double radius = r1 + (r2 - r1) * drand48();
            double theta = orientation + (0.1 - 0.2 * drand48());
            Eigen::Vector3d pos3d = nodePos + Eigen::Vector3d(-radius * cos(theta), -radius * sin(theta), 0.0);


            //std::cout << o_particles[i].Weight()<< std::endl;
            while (!o_occMap->IsValid(pos3d.head(2)))
            {
            	double chance = drand48();
            	radius = r1 + (r2 - r1) * drand48();
            	theta = orientation + (0.1 - 0.2 * drand48());
            	if (chance < 0.05)
            	{
            		int ind = rand() % o_numParticles;
            		int currNodeID = o_particles[ind].ID();
            		nodePos = o_navGraph->Node(currNodeID)->Pos(); 
            	}
            	pos3d = nodePos + Eigen::Vector3d(-radius * cos(theta), -radius * sin(theta), 0.0);

            	//std::cout << currNodeID << "," << pos3d << std::endl;
            }

            orientation +=  0.1 - 0.2 *  drand48();
            Eigen::Vector4d pose(pos3d(0), pos3d(1), pos3d(2), orientation);

            new_particles[j] = Particle(pose, currNodeID, 1.0);
		}

		o_particles = new_particles;
		std::cout << "resampled" << std::endl;
	}
}



/*
void MCL::dumpParticles()
{
	std::string path =  std::to_string(o_frame) + "_particles.csv";
	std::ofstream particleFile;
    particleFile.open(path, std::ofstream::out);
    particleFile << "x" << "," << "y" << "," << "yaw" << "," << "w" << std::endl;
    for(long unsigned int p = 0; p < o_particles.size(); ++p)
    {
        Eigen::Vector3d pose = o_particles[p].pose;
        double w = o_particles[p].weight;
        particleFile << pose(0) << "," << pose(1) << "," << pose(2) << "," << w << std::endl;
    }
    particleFile.close();
    ++o_frame;
}




void MCL::Recover()
{
	o_particleFilter->InitUniform(o_particles, o_numParticles);
}

*/



 


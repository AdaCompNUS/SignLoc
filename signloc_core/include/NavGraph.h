/**
# ##############################################################################
#  Copyright (c) 2025 - National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: NavGraph.h      			                   		       #
# ##############################################################################
**/


#ifndef NAVGRAPH_H
#define NAVGRAPH_H

#include <eigen3/Eigen/Dense>
#include <pybind11/embed.h>
#include <unordered_map>
#include <memory>
#include <tuple>

class NavGraphNode
{
	public: 
		NavGraphNode(int id, Eigen::Vector3d& pos, std::string& type, std::string name=std::string(""), std::string category=std::string(""));
		//NavGraphNode(const NavGraphNode& other);

		std::string Name()
		{
			return o_name;
		}

		std::string Type()
		{
			return o_type;
		}

		Eigen::Vector3d Pos()
		{
			return o_pos;
		}

	int o_id = -1;
	std::string o_name;
	std::string o_type;
	std::string o_category;
	Eigen::Vector3d o_pos;		
};

class NavGraphEdge
{
	public: 
		NavGraphEdge(int u, int v, std::string& type, double weight=0);

		std::string Type()
		{
			return o_type;
		}

		float Weight()
		{
			return o_weight;
		}

	int o_u;
	int o_v;
	std::string o_type;
	double o_weight;		
};



typedef std::unordered_map<int, std::unordered_map<int, std::shared_ptr<NavGraphEdge>>> EdgeDict;


class NavGraph 
{

	public: 
		NavGraph(const std::string& modulePath, const std::string& NavGraphPath);
		

		// std::shared_ptr<DistributionData> Compress(const std::vector<Particle>& particles, Eigen::Vector3f trans);

		// Eigen::MatrixXi Compress(Eigen::MatrixXd& data);


		// void Fuse(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);

		// void ReciprocalSampling(std::vector<Particle>& particles, std::shared_ptr<DistributionData>& distData);

		const std::unordered_map<int, std::shared_ptr<NavGraphNode>>& Nodes()
		{
			return o_nodes;
		}


		std::unordered_map<int, std::shared_ptr<NavGraphNode>> FindNodesOfType(const std::vector<std::string>& nodeTypes);

		std::unordered_map<int, std::unordered_map<int, std::shared_ptr<NavGraphEdge>>> FindEdgesOfType(const std::vector<std::string>& edgeTypes);


		std::tuple<int, double> FindClosestNeighborNode(int nodeID, Eigen::Vector4d pose);


		std::tuple<int, double> FindClosestNode(Eigen::Vector4d pose);


		std::unordered_map<int, std::shared_ptr<NavGraphNode>> FindConnectedNodesOfType(int nodeID, const std::vector<std::string>& nodeTypes);

		std::unordered_map<int, std::unordered_map<int, std::shared_ptr<NavGraphEdge>>> FindConnectedEdgesOfType(int nodeID, const std::vector<std::string>& edgeTypes);


		std::shared_ptr<NavGraphNode>& Node(int id)
		{
			return o_nodes[id];
		}

		std::unordered_map<int, std::shared_ptr<NavGraphEdge>>& Edges(int id)
		{
			return o_edges[id];
		}

		std::shared_ptr<NavGraphEdge>& Edge(int u, int v)
		{
			return o_edges[u][v];
		}



		std::vector<int>& Path(int u, int v)
		{
			return o_paths[u][v];
		}



	private:

		pybind11::object load_graph;
		pybind11::object dc;
		std::unordered_map<int, std::shared_ptr<NavGraphNode>> o_nodes; 
		std::unordered_map<int, std::unordered_map<int, std::shared_ptr<NavGraphEdge>>> o_edges; 
		std::unordered_map<int, std::unordered_map<int, double>> o_distances; 
		std::unordered_map<int, std::unordered_map<int, std::vector<int>>> o_paths; 
		
};

#endif
/**
# ##############################################################################
#  Copyright (c) 2025 - National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: NavGraph.cpp    			                   		       #
# ##############################################################################
**/

#include "NavGraph.h"
#include <pybind11/embed.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <list>
#include <algorithm>
#include <cfloat>



namespace py = pybind11;
using namespace py::literals;




NavGraphNode::NavGraphNode(int id, Eigen::Vector3d& pos, std::string& type, std::string name, std::string category)
{
	o_id = id;
	o_name = name;
	o_type = type;
	o_category = category;
	o_pos = pos;
}

NavGraphEdge::NavGraphEdge(int u, int v, std::string& type, double weight)
{
	o_u = u;
	o_v = v;
	o_type = type;
	o_weight = weight;
}



std::string removeJunk(const std::string& input)
{
	std::string str = input;
	char chars[] = "[],'";
	for (unsigned int i = 0; i < strlen(chars); ++i)
   {
      // you need include <algorithm> to use general algorithms like std::remove()
      str.erase (std::remove(str.begin(), str.end(), chars[i]), str.end());
   }

   return str;
}

NavGraph::NavGraph(const std::string& modulePath, const std::string& NavGraphPath)
{
	try {
		py::initialize_interpreter();
	}
	catch (...) {
	  std::cout << "Interpreter already initialized!" << std::endl;
	}
	py::module sys = py::module::import("sys");
	py::module np = py::module::import("numpy");

	py::list path = sys.attr("path");
	path.attr("append")(modulePath);

	dc = py::module::import("nx_graph");
	load_graph = dc.attr("load_graph");
	py::object d = load_graph(NavGraphPath);


	std::cout.precision(10);


	py::list nodeList = d["nodes"];
	py::list dataList = d["nodesData"];
	py::list edgeList = d["edges"];
	py::list edgeDataList = d["edgesData"];
	py::dict distanceList = d["distances"];
	py::dict pathList = d["paths"];



	for (auto item : distanceList) 
	{
		py::handle uKey = item.first;
        py::handle uDict = item.second;

        int u = uKey.cast<int>();
        py::dict distanceList4u = py::cast<py::dict>(uDict);
        for (auto item2 : distanceList4u) 
		{
	        py::handle vKey = item2.first;
        	py::handle vVal = item2.second;
        	int v = vKey.cast<int>();
        	double dist = vVal.cast<double>();
        	//std::cout<< u << ", " << v << ", " << dist << std::endl;

        	o_distances[u][v] = dist;
    	}
    }


    for (auto item : pathList) 
	{
		py::handle uKey = item.first;
      py::handle uDict = item.second;

      int u = uKey.cast<int>();
      py::dict distanceList4u = py::cast<py::dict>(uDict);
      for (auto item2 : distanceList4u) 
		{
	      py::handle vKey = item2.first;
        	py::handle vVal = item2.second;
        	int v = vKey.cast<int>();
        	
        	Eigen::MatrixXi paths_ = vVal.cast<Eigen::MatrixXi>();
        	std::vector<int> path; 

        	for(int r = 0; r < paths_.rows(); ++r)
        	{
        		//std::cout << paths_(r, 0)  << std::endl;
        		path.push_back(paths_(r, 0));
        	}
        	o_paths[u][v] = path;
    	}
    }

	for (size_t i = 0; i < nodeList.size(); ++i)
    {
    	int nodeID = nodeList[i].cast<int>();
    	py::dict nodeData = dataList[i];
    	Eigen::MatrixXd pos_ = nodeData["pos"].cast<Eigen::MatrixXd>();
    	Eigen::Vector3d pos = Eigen::Vector3d(pos_);
    	   
    	//std::cout<< pos << std::endl;

    	std::string type = nodeData["type"].cast<std::string>();
    	std::string name = "";
    	if (nodeData.contains("name"))
    	{
    		py::str stmp = nodeData["name"];
    		name = removeJunk(stmp.cast<std::string>());
    	}
    	std::string category = "";
    	if (nodeData.contains("category"))
    	{
    		py::str ctmp = nodeData["category"];
    		category = ctmp.cast<std::string>();
    	}

    	std::shared_ptr<NavGraphNode> ng = std::make_shared<NavGraphNode>(nodeID, pos, type, name, category);
    	o_nodes[nodeID] = ng;

    }

    for (size_t i = 0; i < edgeList.size(); ++i)
    {
    	std::tuple<int, int> nodeIDs = edgeList[i].cast<std::tuple<int, int>>();
    	int u = std::get<0>(nodeIDs);
    	int v = std::get<1>(nodeIDs);
    	py::dict edgeData = edgeDataList[i];
    	std::string type = edgeData["type"].cast<std::string>();
    	double weight = 0;
    	if (edgeData.contains("weight"))
    	{
    		weight = edgeData["weight"].cast<double>();
    	}

    	std::shared_ptr<NavGraphEdge> ne = std::make_shared<NavGraphEdge>(u, v, type, weight);
    	o_edges[u][v] = ne;
    }
}


std::unordered_map<int, std::shared_ptr<NavGraphNode>> NavGraph::FindNodesOfType(const std::vector<std::string>& nodeTypes)
{

	std::unordered_map<int, std::shared_ptr<NavGraphNode>> specificNodes;

    for (auto item : o_nodes) 
	{
		int nodeID = item.first;
        std::shared_ptr<NavGraphNode> node = item.second;


        bool found = (std::find(nodeTypes.begin(), nodeTypes.end(), node->Type()) != nodeTypes.end());
        if (found)
        {
        	specificNodes[nodeID] = node;
        }
    }

    return specificNodes;
}

std::unordered_map<int, std::shared_ptr<NavGraphNode>> NavGraph::FindConnectedNodesOfType(int nodeID, const std::vector<std::string>& nodeTypes)
{
	std::unordered_map<int, std::shared_ptr<NavGraphNode>> specificNodes;

	std::unordered_map<int, std::shared_ptr<NavGraphEdge>> connectedEdges = this->Edges(nodeID);
	for (auto item : connectedEdges) 
	{
		int v = item.first;
		std::shared_ptr<NavGraphNode> node = this->Node(v);

		bool found = (std::find(nodeTypes.begin(), nodeTypes.end(), node->Type()) != nodeTypes.end());
		if (found)
		{
			specificNodes[v] = node;
		}
	}

	return specificNodes;
}

std::unordered_map<int, std::unordered_map<int, std::shared_ptr<NavGraphEdge>>> NavGraph::FindConnectedEdgesOfType(int nodeID, const std::vector<std::string>& edgeTypes)
{
	std::unordered_map<int, std::unordered_map<int, std::shared_ptr<NavGraphEdge>>> specificEdges;

	std::unordered_map<int, std::shared_ptr<NavGraphEdge>> connectedEdges = this->Edges(nodeID);
	for (auto item : connectedEdges) 
	{
		int v = item.first;
		std::shared_ptr<NavGraphEdge> edge =item.second;

		bool found = (std::find(edgeTypes.begin(), edgeTypes.end(), edge->Type()) != edgeTypes.end());
		if (found)
		{
			specificEdges[nodeID][v] = edge;
		}
	}

	return specificEdges;
}



std::unordered_map<int, std::unordered_map<int, std::shared_ptr<NavGraphEdge>>> NavGraph::FindEdgesOfType(const std::vector<std::string>& edgeTypes)
{
	std::unordered_map<int, std::unordered_map<int, std::shared_ptr<NavGraphEdge>>> specificEdges;

	for (auto item : o_edges) 
	{
		int u = item.first;
		std::unordered_map<int, std::shared_ptr<NavGraphEdge>> uEdges = item.second;

		for (auto item2 : uEdges) 
		{
			int v = item2.first;
			std::shared_ptr<NavGraphEdge> edge = item2.second;

			bool found = (std::find(edgeTypes.begin(), edgeTypes.end(), edge->Type()) != edgeTypes.end());
			if (found)
			{
				specificEdges[u][v] = edge;
			}
		}
	}

	return specificEdges;
}


std::tuple<int, double> NavGraph::FindClosestNode(Eigen::Vector4d pose)
{
	double minDist = DBL_MAX;
	int minDistID = -1;

	for (auto item : o_nodes) 
	{
		int nodeID = item.first;
        std::shared_ptr<NavGraphNode> node = item.second;

		double dist = (pose.head(3) - node->Pos()).norm();
		if (dist < minDist)
		{
			minDist = dist;
			minDistID = nodeID;
		}
	}

	std::tuple<int, double> result = std::make_tuple(minDistID, minDist);

	return result;
}


std::tuple<int, double> NavGraph::FindClosestNeighborNode(int nodeID, Eigen::Vector4d pose)
{
	std::vector<std::string> edgeTypes = {"action"};

	std::shared_ptr<NavGraphNode> currNode = this->Node(nodeID);
	Eigen::Vector3d currPos = currNode->Pos();
	std::unordered_map<int, std::shared_ptr<NavGraphEdge>> connectedEges = this->Edges(nodeID);

	double minDist = (pose.head(3) - currPos).norm();
	int minDistID = nodeID;

	for (auto item : connectedEges) 
	{
		int v = item.first;
		std::shared_ptr<NavGraphEdge> edge = item.second;
		//if (this->Node(v)->Type() == "portal") continue;

		bool found = (std::find(edgeTypes.begin(), edgeTypes.end(), edge->Type()) != edgeTypes.end());
		if (found)
		{
			Eigen::Vector3d neighborPos = this->Node(v)->Pos();
			double dist = (pose.head(3) - neighborPos).norm();
			if (dist < minDist)
			{
				minDist = dist;
				minDistID = v;
			}
		}
	}

	std::tuple<int, double> result = std::make_tuple(minDistID, minDist);

	return result;

}





/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: NavOccMap.cpp                                                               #
# ##############################################################################
**/

#include "NavOccMap.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sstream>
#include "Utils.h"
#include <unordered_map>
#include <set>

NavOccMap::NavOccMap(std::vector<std::vector<Eigen::Vector3d>> edges, Eigen::Vector2d origin, double resolution, int radius)
{
	o_resolution = resolution;
	o_origin = origin;

	o_occMap = cv::Mat::zeros(radius, radius, CV_8UC1);
	o_height = o_occMap.rows;
	o_width = o_occMap.cols;

	// std::set<int> heightMapSet;
	// for (int i = 0; i < edges.size(); ++i)
	// {
	// 	Eigen::Vector3d p1 = edges[i][0];
	// 	Eigen::Vector3d p2 = edges[i][1];
	// 	heightMapSet.insert(int(p1(2)));
	// 	heightMapSet.insert(int(p2(2)));
	// }

	// int numMaps = heightMapSet.size();
	// std::vector<int> heightMap(heightMapSet.begin(), heightMapSet.end());
	// for (int i = 0; i < heightMap.size(); ++i)
	// {
	// 	cv::Mat tmpMap = cv::Mat::zeros(radius, radius, CV_8UC1); 
	// 	o_occMaps.push_back(tmpMap);
	// }

	// for(size_t i = 0; i < edges.size(); ++i)
	// {
	// 	Eigen::Vector3d p1 = edges[i][0]; 
	// 	Eigen::Vector3d p2 = edges[i][1];


	// 	auto it = std::find(heightMap.begin(), heightMap.end(), int(p1(2)));
	// 	int ind1 = std::distance(heightMap.begin(), it);
	// 	auto it2 = std::find(heightMap.begin(), heightMap.end(), int(p2(2)));
	// 	int ind2 = std::distance(heightMap.begin(), it2);

	// 	Eigen::Vector2i u = this->World2Map(p1.head(2));
	// 	Eigen::Vector2i v = this->World2Map(p2.head(2));

	// 	cv::line(o_occMaps[ind1], cv::Point(u[0], u[1]) , cv::Point(v[0], v[1]), 255, 1);
	// 	cv::line(o_occMaps[ind2], cv::Point(u[0], u[1]) , cv::Point(v[0], v[1]), 255, 1);
	// }


	cv::Mat tmpMap = cv::Mat::zeros(radius, radius, CV_8UC1);
	for(size_t i = 0; i < edges.size(); ++i)
	{
		Eigen::Vector3d p1 = edges[i][0];
		Eigen::Vector3d p2 = edges[i][1];

		Eigen::Vector2i u = this->World2Map(p1.head(2));
		Eigen::Vector2i v = this->World2Map(p2.head(2));

		cv::line(tmpMap, cv::Point(u[0], u[1]) , cv::Point(v[0], v[1]), 255, 1);
	}
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(tmpMap, o_occMap, kernel); 
  	cv::imwrite("/GraphLoc/ros_ws/COccMap.png", o_occMap);

  	std::cout << "Map created!" << std::endl;
}


//p = pos[:2] - self.center 
//uv = np.array([self.w * 0.5 + self.res * p[0], self.h * 0.5 - self.res * p[1]]).astype(np.int32)

Eigen::Vector2i NavOccMap::World2Map(Eigen::Vector2d xy) const
{
	//std::cout << xy << std::endl;

	Eigen::Vector2d relDist = xy - o_origin;
	double u = 0.5 * o_width + relDist(0) * o_resolution;
	double v = 0.5 * o_height - (relDist(1) * o_resolution);

	return Eigen::Vector2i(u, v);
}


bool NavOccMap::IsValid(Eigen::Vector2d xy) const
{
	Eigen::Vector2i mp = this->World2Map(xy);

	//std::cout << mp << std::endl;

	if ((mp(0) < 0) || (mp(1) < 0) || (mp(0) >= o_width) || (mp(1) >= o_height)) return false;

	int val = o_occMap.at<uchar>(int(mp(1)), int(mp(0)));

	if (val == 0) return false;

	return true;
}


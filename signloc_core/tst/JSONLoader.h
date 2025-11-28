
/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: JSONLoader.h                                                           #
# ##############################################################################
**/

#ifndef JSONLOADER_H
#define JSONLOADER_H

#include <pybind11/embed.h>#include <pybind11/embed.h>
#include <unordered_map>
#include <tuple>
#include <eigen3/Eigen/Dense>

typedef std::unordered_map<std::string, std::vector<std::tuple<int, double>>> MiniGraph;


class JSONLoader 
{

	public: 
		JSONLoader(const std::string& modulePath, const std::string& configPath);

		MiniGraph GetPrediction(const std::string& framePath);

		Eigen::Vector4d GetGT(const std::string& framePath);





	private:
		std::unordered_map<std::string, MiniGraph> o_predictionList;
		std::unordered_map<std::string, Eigen::Vector4d> o_GTList;


};

#endif

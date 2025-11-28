/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: JSONLoader.cpp                                                        #
# ##############################################################################
**/



#include "JSONLoader.h"
#include <iostream>
#include <pybind11/embed.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <iostream>
#include <list>
#include <algorithm>



namespace py = pybind11;
using namespace py::literals;

JSONLoader::JSONLoader(const std::string& modulePath, const std::string& configPath)
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

	py::object dc = py::module::import("load_json");
	py::object loadPredictions = dc.attr("loadPredictions");
	py::object loadGT = dc.attr("loadGT");
	py::dict predList = loadPredictions(configPath);
	py::dict GTList = loadGT(configPath);

	for (auto item : predList) 
	{
        py::dict pred = py::cast<py::dict>(item.second);
        std::string framePath = item.first.cast<std::string>();
        MiniGraph minigraph;

        for (auto item2 : pred)
        {
        	std::string place = item2.first.cast<std::string>();
        	//std::cout <<  place << std::endl;
        	Eigen::MatrixXd actionDistMat = item2.second.cast<Eigen::MatrixXd>();
        	std::vector<std::tuple<int, double>> actionDist;

        	for(int r = 0; r < actionDistMat.rows(); ++r)
        	{
        		//std::cout << actionDistMat(r, 0) << ", " << actionDistMat(r, 1)  << std::endl;
        		std::tuple<int, double> t = std::make_tuple<int, double>(actionDistMat(r, 0), double(actionDistMat(r, 1)));
        		actionDist.push_back(t);
        	}
        	minigraph[place] = actionDist;
        }
        o_predictionList[framePath] = minigraph;
     }

    for (auto item : GTList) 
	{
        std::string framePath = item.first.cast<std::string>();
        Eigen::MatrixXd gt = Eigen::Vector4d(item.second.cast<Eigen::MatrixXd>().data());
      	o_GTList[framePath] = gt;
	}

}



MiniGraph JSONLoader::GetPrediction(const std::string& framePath)
{
	MiniGraph mini = o_predictionList[framePath];

	return mini;
}

Eigen::Vector4d JSONLoader::GetGT(const std::string& framePath)
{
	return o_GTList[framePath];
}

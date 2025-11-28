/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: NavOccMap.h                                                                 #
# ##############################################################################
**/

#ifndef NAVOCCMAP
#define NAVOCCMAP

#include <memory>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>

class NavOccMap 
{
	public:

		//! A constructor for handling the output of Gmapping, which include a metadata yaml and a .pgm map 
	    /*!
	      \param origin is the 2D pose of the bottom right corner of the map (found in the yaml)
	      \param resolution is the map resolution - distance in meters corresponding to 1 pixel (found in the yaml)
	      \param gridMap is occupancy map built according to the scans
	    */
		NavOccMap(std::vector<std::vector<Eigen::Vector3d>> edges, Eigen::Vector2d origin, double resolution, int radius);


		//! Converts (x, y) from the map frame to the pixels coordinates
		/*!
			\param (x, y) position in map frame
		   \return (u, v) pixel coordinates for the gridmap
		*/
		Eigen::Vector2i World2Map(Eigen::Vector2d xy) const;


		bool IsValid(Eigen::Vector2d xy) const;



	private:

		std::vector<cv::Mat> o_occMaps;
		cv::Mat o_occMap;
		double o_resolution = 0;
		Eigen::Vector2d o_origin;
		int o_width;
		int o_height;		
};

#endif //GMap


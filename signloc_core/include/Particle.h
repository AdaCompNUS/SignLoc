/**
# ##############################################################################
#  Copyright (c) 2024- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Particle.h                                                            #
# ##############################################################################
**/

#ifndef PARTICLE_H
#define PARTICLE_H


#include <vector>
#include <eigen3/Eigen/Dense>


class Particle
{
public:

	Particle(Eigen::Vector4d p = Eigen::Vector4d(0, 0, 0, 0), int nodeID = -1, double w = 1.0)
	{
		o_pose = p;
		o_nodeID = nodeID;
		o_weight = w;
	}



	int ID() const
	{
		return o_nodeID;
	}

	Eigen::Vector4d Pose() const
	{
		return o_pose;
	}

	double Weight() const
	{
		return o_weight;
	}

	void Pose(Eigen::Vector4d pose) 
	{
		o_pose = pose;
	}

	void Weight(double w) 
	{
		o_weight = w;
	}

	void ID(int nodeID) 
	{
		o_nodeID = nodeID;
	}



	
	Eigen::Vector4d o_pose;
	int o_nodeID;
	double o_weight;

};


#endif
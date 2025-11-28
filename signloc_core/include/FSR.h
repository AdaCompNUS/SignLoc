/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: FSR.h                                                                 #
# ##############################################################################
**/


#ifndef FSR_H
#define FSR_H

#include <eigen3/Eigen/Dense>
#include <vector>

class FSR 
{
	public:

		Eigen::Vector4d SampleMotion(const Eigen::Vector4d& p1, const Eigen::Vector4d& command, const Eigen::Vector4d& noise);

		Eigen::Vector4d Forward(Eigen::Vector4d p1, Eigen::Vector4d u);

	    Eigen::Vector4d Backward(Eigen::Vector4d p1, Eigen::Vector4d p2);


};

#endif
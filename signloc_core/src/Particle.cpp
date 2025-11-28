/**
# ##############################################################################
#  Copyright (c) 2024- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Particle.cpp                                                               #
# ##############################################################################
**/


#include "Particle.h"

Particle::Particle(Eigen::Vector3d p, double w)
{
	pose = p;
	weight = w;
}
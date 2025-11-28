/**
# ##############################################################################
#  Copyright (c) 2024- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: FSR.cpp                                                               #
# ##############################################################################
**/

#include "FSR.h"
#include <math.h>
#include <stdlib.h>
#include "Utils.h"
#include <iostream>



Eigen::Vector4d FSR::SampleMotion(const Eigen::Vector4d& p1, const Eigen::Vector4d& u, const Eigen::Vector4d& noise)
{

	double f = u(0);
	double s = u(1);
	double z = u(2);
	double r = u(3);

	double f_h = f - SampleGuassian(noise(0) * fabs(f));
	double s_h = s - SampleGuassian(noise(1) * fabs(s));
	double z_h = z - SampleGuassian(noise(2) * fabs(z));
	double r_h = r - SampleGuassian(noise(3) * fabs(r));


	Eigen::Vector4d new_p = Forward(p1, Eigen::Vector4d(f_h, s_h, z_h, r_h));

	return new_p;

}


Eigen::Vector4d FSR::Backward(Eigen::Vector4d p1, Eigen::Vector4d p2)
{
	Eigen::Vector4d dp = p2 - p1;

	double a = cos(p1(3));
	double b = sin(p1(3));
	
    double f = (dp.x() * a + dp.y() * b) /  (pow(a, 2.0) + pow(b, 2.0));
    double s = (dp.y() - f * b) / a;
    double r = dp(3);
    double h = dp(2);


	return Eigen::Vector4d(f, s, h, r);

}

Eigen::Vector4d FSR::Forward(Eigen::Vector4d p1, Eigen::Vector4d u)
{
	double f = u(0);
	double s = u(1);
	double h = u(2);
	double r = u(3);

	double x = p1(0) + f * cos(p1(3)) - s * sin(p1(3));
	double y = p1(1) + f * sin(p1(3)) + s * cos(p1(3));
	double theta = WrapTo2Pi(r + p1(3));
	double z = p1(2) + h;

	return Eigen::Vector4d(x, y, z, theta);

}




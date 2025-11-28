/**
# ##############################################################################
#  Copyright (c) 2024- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Utils.cpp                                                             #
# ##############################################################################
**/

#include <math.h>
#include "Utils.h"
#include <vector>
#include <algorithm>
#include <numeric>
#include <iterator>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdlib.h>



Eigen::Matrix3d Vec2Trans(Eigen::Vector3d v)
{
	double c = cos(v(2));
	double s = sin(v(2));
	Eigen::Matrix3d trans;
	trans << c, -s, v(0),	s, c, v(1),	0, 0, 1;

	return trans;
}

double CosineSimilarity(double yaw1, double yaw2)
{
	Eigen::Vector2d unitVec1 = Eigen::Vector2d(cos(yaw1), sin(yaw1));
	Eigen::Vector2d unitVec2 = Eigen::Vector2d(cos(yaw2), sin(yaw2));

	double sim = unitVec1.dot(unitVec2);

	return sim;
}

double Wrap2Pi(double angle)
{
	double wAngle = angle;
	while (wAngle < -M_PI) wAngle += 2 * M_PI;

	while (wAngle > M_PI) wAngle -= 2 * M_PI;

	return wAngle;
}

double WrapTo2Pi(double angle)
{
	double wAngle = angle;
	while (wAngle < 0.0) wAngle += 2 * M_PI;

	while (wAngle >= 2*M_PI) wAngle -= 2 * M_PI;

	return wAngle;

}

double GetYaw(double qz, double qw)
{
	double yaw = 2 * atan2(qz, qw);
	yaw = Wrap2Pi(yaw);

	return yaw;
}

double SampleGuassian(double sigma)
{
	double sample = 0;

	for(int i = 0; i < 12; ++i)
	{
		sample += drand48() * 2 * sigma - sigma;
	}
	sample *= 0.5;

	return sample;
}

int rad2action(double rad, int actionNum)
{
	double angle = WrapTo2Pi(rad + 0.5 * M_PI + M_PI / actionNum);
	int action = int(actionNum * angle/ (2 * M_PI));

	return action;
}

double Vec2deg(Eigen::Vector2d dirc)
{
	double deg = -0.5 *  M_PI  + atan2(dirc(1), dirc(0));
	return deg;
}


/*
void DumpParticles(std::vector<Particle>& particles, std::string& path)
{
	std::ofstream particleFile;
    particleFile.open(path, std::ofstream::out);
    //particleFile << "x" << "," << "y" << "," << "yaw" << "," << "w" << std::endl;
    for(long unsigned int p = 0; p < particles.size(); ++p)
    {
        Eigen::Vector3d pose = particles[p].pose;
        double w = particles[p].weight;
        particleFile << pose(0) << " " << pose(1) << " " << pose(2) << " " << w << std::endl;
    }
    particleFile.close();

}

void DumpGPS(Eigen::Vector4d gps, std::string& path)
{
	std::ofstream gpsFile;
    gpsFile.open(path, std::ofstream::out);

    gpsFile << gps(0) << " " << gps(1) << " " << gps(2) << " " << gps(3) << std::endl;
    gpsFile.close();
}


void DrawParticles(cv::Mat& map, std::vector<Eigen::Vector3d>& uvs, std::string& path)
{
	cv::Mat draw;
	cv::cvtColor(map, draw, cv::COLOR_GRAY2RGB);

	for(int i = 0; i < uvs.size(); i++)
	{
		cv::Point p(int(uvs[i](0)), int(uvs[i](1)));
		cv::circle(draw, p, 2, cv::Scalar( 0, 0, 255 ), cv::FILLED, cv::LINE_8 );

		double dx = sin(uvs[i](2));
		double dy = cos(uvs[i](2));
		cv::Point p2(p.x + 5 * dx, p.y + 5 * dy);

		cv::arrowedLine(draw, p, p2, cv::Scalar( 180, 0, 255 ));
	}

	cv::imwrite(path, draw);
}
*/


double action2rad(int action, int actionNum)
{
	double rad = -0.5 * M_PI + action * 2 * M_PI / actionNum;
	return rad;
}

Eigen::Vector2d rad2Vec(double rad)
{

	Eigen::Vector2d dirc = Eigen::Vector2d(-sin(rad), cos(rad)).normalized();
	return dirc;
}


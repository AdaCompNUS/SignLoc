/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: Utils.h                                                               #
# ##############################################################################
**/

#ifndef UTILS_H
#define UTILS_H


#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include "Particle.h"
#include <opencv2/opencv.hpp>

Eigen::Matrix3d Vec2Trans(Eigen::Vector3d v);

double CosineSimilarity(double yaw1, double yaw2);

double Wrap2Pi(double angle);

double WrapTo2Pi(double angle);

double GetYaw(double qz, double qw);

double SampleGuassian(double sigma);

/*void DumpParticles(std::vector<Particle>& particles, std::string& path);

void DrawParticles(cv::Mat& map, std::vector<Eigen::Vector3d>& uvs, std::string& path);

void DumpGPS(Eigen::Vector4d gps, std::string& path);
*/

double action2rad(int action, int actionNum);

int rad2action(double rad, int actionNum);

double Vec2deg(Eigen::Vector2d);

Eigen::Vector2d rad2Vec(double rad);


#endif
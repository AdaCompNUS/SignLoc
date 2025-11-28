/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: MCL.h                                                                 #
# ##############################################################################
**/

#ifndef MCL_H
#define MCL_H


#include <pybind11/embed.h>


#include "FSR.h"
#include "Particle.h"
#include "NavOccMap.h"
#include <memory>
#include "NavGraph.h"
#include <unordered_map>
#include <tuple>
#include "ActionMotionModel.h"


typedef std::vector<std::tuple<std::string, int, double>> NodeDist;
typedef std::vector<std::tuple<int, double>> ActionDist;
typedef std::tuple<std::vector<NodeDist>, std::vector<ActionDist>> GraphLocData;
typedef std::unordered_map<std::string, std::vector<std::tuple<int, double>>> MiniGraph;


class MCL
{
	public:


		//! A constructor
	    /*!
	     \param fm is a ptr to a GMap object
	      \param mm is a ptr to a MotionModel object, which is an abstract class. FSR is the implementation 
	      \param sm is a ptr to a BeamEnd object, which is an abstract class. BeamEnd is the implementation 
	      \param rs is a ptr to a Resampling object, which is an abstract class. LowVarianceResampling is the implementation 
	      \param n_particles is an int, and it defines how many particles the particle filter will use
	    */
		MCL(const std::string& NavGraphPath, int n_particles, Eigen::Vector2d center, bool ros=true, double sigma=1.0);


		//! A constructor
	    /*!
	     \param fm is a ptr to a GMap object
	    */
		//MCL(const std::string& jsonConfigPath);


		//! A constructor
	    /*!
	     * \param fm is a ptr to a GMap object
	      \param mm is a ptr to a MotionModel object, which is an abstract class. FSR is the implementation 
	      \param sm is a ptr to a BeamEnd object, which is an abstract class. BeamEnd is the implementation 
	      \param rs is a ptr to a Resampling object, which is an abstract class. LowVarianceResampling is the implementation 
	      \param n_particles is an int, and it defines how many particles the particle filter will use
	      \param initGuess is a vector of initial guess for the location of the robots
	      \param covariances is a vector of covariances (uncertainties) corresponding to the initial guesses
	      \param injectionRatio is an float, and it determines which portions of the particles are replaced when relocalizing
	    */
		// MCL(std::shared_ptr<GMap> gmap, std::shared_ptr<FSR> mm, std::shared_ptr<BeamEnd> sm, 
		// 	std::shared_ptr<Resampling> rs, int n_particles, std::vector<Eigen::Vector3d> initGuess, 
		// 	std::vector<Eigen::Matrix3d> covariances);


		
		//! A getter particles representing the pose hypotheses 
		/*!
		   \return A vector of points, where each is Eigen::Vector3d = (x, y, theta)
		*/
		std::vector<Particle> Particles()
		{
			return o_particles;
		}


		void InitParticles();

		//void InitParticles(cv::Mat& canvas, Eigen::Vector3d gps, const std::vector<Eigen::Vector3d>& initGuess, const std::vector<Eigen::Matrix3d>& covariances);




		//! Advanced all particles according to the control and noise, using the chosen MotionModel's forward function
		/*!
		  \param control is a 3d control command. In the FSR model it's (forward, sideways, rotation)
		  \param odomWeights is the corresponding weight to each odometry source
	      \param noise is the corresponding noise to each control component
		*/
		void Predict(Eigen::Vector4d& control, const Eigen::Vector4d& noise);


		void PredictAction(Eigen::Vector4d& p1, const Eigen::Vector4d& p2);

		//! Considers the beamend likelihood of observation for all hypotheses, and then performs resampling 
		/*!
		  \param scan is a vector of homogeneous points (x, y, 1), in the sensor's frame. So the sensor location is (0, 0, 0)
		  		Notice that for the LaserScan messages you need to first transform the ranges to homo points, and then center them 
		*/
		void Correct(const MiniGraph& minigraph);

		GraphLocData ParseMiniGraph(const MiniGraph& minigraph);


		void NormalizeWeights();


		void Resampling();


		//! Initializes filter with new particles upon localization failure
		//void Recover();

		

		Eigen::Vector4d Backward(Eigen::Vector4d p1, Eigen::Vector4d p2)
		{
			return o_motionModel->Backward(p1, p2);
		}

		std::vector<Particle> Particles() const
		{
			return o_particles;
		}


		std::shared_ptr<NavGraph>& Graph() 
		{
			return o_navGraph;
		}


		void SetLikelihoodSigma(double sigma)
		{
			o_sigma = sigma;
		}

	private:

		void dumpParticles();

		void createOccupancyGrid();

		Particle initParticleUniform();

		Particle initParticleFromNodeID(int nodeID);


		std::shared_ptr<NavGraph> o_navGraph;
		unsigned int o_frame = 0;
		double o_sigma = 0.5;
		int o_actionNum = 8;
		int o_orientationNum = 16;
		int o_nodeNum;
		double o_efficiencyCoeff = 0.5;
		bool o_ros;
		double o_penalty = 0.00001;

		std::shared_ptr<ActionMotionModel>  o_actionMotionModel;

		std::shared_ptr<FSR> o_motionModel;
		std::shared_ptr<NavOccMap> o_occMap;
		int o_numParticles = 0;
		Eigen::Vector2d o_gpsCenter;
		std::vector<Particle> o_particles;

		std::vector<std::string> o_placeNodeNames;
		std::vector<int> o_placeNodesIDs;
		std::unordered_map<int, std::shared_ptr<NavGraphNode>> o_traverseNodes;
		std::vector<int> o_traverseNodeIDs;
		//o_traverseNodes;

		pybind11::object o_matchLabels;
		pybind11::object o_fuzzyWrapper;

};

#endif

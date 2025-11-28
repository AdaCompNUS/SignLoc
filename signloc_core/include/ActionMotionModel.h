/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ActionMotionModel.h                                                           #
# ##############################################################################
**/


#ifndef ACTIONMOTIONMODEL_H
#define ACTIONMOTIONMODEL_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include "NavGraph.h"

typedef  std::tuple<std::vector<int>, double> Control;
typedef  std::tuple<int, double> State;

class ActionMotionModel
{
    public:

        ActionMotionModel(std::shared_ptr<NavGraph> navGraph, int actionNum);

        //Eigen::Vector4d Forward(Eigen::Vector4d p1, Eigen::Vector4d u);

        Control Backward(Eigen::Vector4d p1, Eigen::Vector4d p2);

        State Forward(State& state, Control& ctrl);

    private:
        std::shared_ptr<NavGraph> o_navGraph;
        int o_actionNum;


};

#endif
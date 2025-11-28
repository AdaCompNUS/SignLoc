/**
# ##############################################################################
#  Copyright (c) 2025- National University of Singapore                        #
#  All rights reserved.                                                        #
#                                                                              #
#  Author: Nicky Zimmerman                                                     #
#                                                                              #
#  File: ActionMotionModel.cpp                                                 #
# ##############################################################################
**/

#include "ActionMotionModel.h"
#include "Utils.h"

ActionMotionModel::ActionMotionModel(std::shared_ptr<NavGraph> navGraph, int actionNum)
{
    o_navGraph = navGraph;
    o_actionNum = actionNum;
}


Control ActionMotionModel::Backward(Eigen::Vector4d p1, Eigen::Vector4d p2)
{
    std::vector<int> actions;

    double currOrientation = p1(3);
    int  nodeID1 =  std::get<0>(o_navGraph->FindClosestNode(p1));
    int  nodeID2 =  std::get<0>(o_navGraph->FindClosestNode(p2));
    std::vector<int> path = o_navGraph->Path(nodeID1, nodeID2);

    // for(size_t i = 0; i < path.size(); ++i)
    // {
    //     std::cout << path[i] << ",";
    // }
    // std::cout << std::endl;

    for(size_t i = 0; i < path.size() - 1; ++i)
    {
        Eigen::Vector3d currPos = o_navGraph->Node(path[i])->Pos();
        Eigen::Vector3d nextPos = o_navGraph->Node(path[i + 1])->Pos();

        Eigen::Vector3d diff = nextPos - currPos;
        if (diff(2) > 0) actions.push_back(9);
        else if (diff(2) < 0) actions.push_back(10);
        else
        {
            Eigen::Vector3d orientationChange = (nextPos - currPos).normalized();
            double o = Vec2deg(orientationChange.head(2));

            //std::cout << orientationChange << ", " << o << ", ";

            int action = rad2action(WrapTo2Pi(o - currOrientation), o_actionNum);
            actions.push_back(action);
            currOrientation = WrapTo2Pi(currOrientation + action2rad(action, o_actionNum));
        }
    }
    //std::cout << std::endl;

    double finalOrientationAdjustment = p2(3) - currOrientation;

    Control ctrl = std::make_tuple(actions, finalOrientationAdjustment);

    return ctrl;

}

State ActionMotionModel::Forward(State& state, Control& ctrl)
{
    std::vector<int> actions = std::get<0>(ctrl);
    double finalOrientationAdjustment = std::get<1>(ctrl);

    int currNodeID = std::get<0>(state);
    double currOrientation = std::get<1>(state);


    std::vector<std::string> edgeTypes = {"action"};

    for(size_t i = 0; i < actions.size(); ++i)
    {
        EdgeDict edgeDict = o_navGraph->FindConnectedEdgesOfType(currNodeID, edgeTypes);
        std::unordered_map<int, std::shared_ptr<NavGraphEdge>> edges = edgeDict[currNodeID];
        int action = actions[i];
        Eigen::Vector3d currPos = (o_navGraph->Node(currNodeID)->Pos());

        if (action > 8)
        {
            bool verticalEdge = false;
            std::vector<int> vIDs;
            for(auto item : edges)
            {
                int v = item.first;
                Eigen::Vector3d nextPossibleDirc = (o_navGraph->Node(v)->Pos() - currPos).normalized();
                if ((action == 9) && (nextPossibleDirc(2) > 0))
                {
                    currNodeID = v;
                    verticalEdge = true;
                    break;
                }
                else if ((action == 10) && (nextPossibleDirc(2) < 0))
                {
                    currNodeID = v;
                    verticalEdge = true;
                    break;
                }
                vIDs.push_back(v);
            }
            if (!verticalEdge)
            {
                // assign a random edge to continue
                int ind =  rand() % vIDs.size();
                currNodeID = vIDs[ind];
            }
        }
        else
        {
            double nextOrientation = WrapTo2Pi(currOrientation + action2rad(action, o_actionNum));
            Eigen::Vector2d nextPredictedDirc = rad2Vec(nextOrientation);
            std::vector<double> angleDiffs;
            std::vector<int> vIDs;
            for(auto item : edges)
            {
                int v = item.first;
                Eigen::Vector3d nextPossibleDirc = (o_navGraph->Node(v)->Pos() - currPos).normalized();

                double angleDiff = acos(nextPossibleDirc.head(2).dot(nextPredictedDirc));
                angleDiffs.push_back(angleDiff);
                vIDs.push_back(v);
            }
            auto min_it = std::min_element(angleDiffs.begin(), angleDiffs.end());
            int minDiffID = std::distance(angleDiffs.begin(), min_it);

            currOrientation = nextOrientation;
            currNodeID = vIDs[minDiffID];
        }
    }

    State nextState = std::make_tuple(currNodeID, WrapTo2Pi(currOrientation + finalOrientationAdjustment));

    return nextState;
}
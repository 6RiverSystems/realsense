/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SOLUTIONGENERATOR_HPP_
#define SOLUTIONGENERATOR_HPP_

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>

#include <srslib_framework/robotics/Pose.hpp>
using namespace srs;

namespace srs {

template<typename GRAPH>
struct SolutionGenerator
{
    static Solution<GRAPH>* fromRotation(Pose<> pose, double theta0, double thetaf)
    {
        SolutionNode<GRAPH> solutionNode;

        solutionNode.actionType = SolutionNode<GRAPH>::ROTATE;

        solutionNode.fromPose = pose;
        solutionNode.toPose = pose;

        solutionNode.fromPose.theta = AngleMath::normalizeAngleRad(theta0);
        solutionNode.toPose.theta = AngleMath::normalizeAngleRad(thetaf);

        return new Solution<GRAPH>(solutionNode);
    }
};

} // namespace srs

#endif // SOLUTIONGENERATOR_HPP_

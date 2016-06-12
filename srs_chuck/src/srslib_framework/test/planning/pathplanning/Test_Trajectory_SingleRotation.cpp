/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
using namespace srs;

typedef SolutionNode<Grid2d> SolutionNodeType;

TEST(Test_Trajectory, SingleRotation)
{
    Chuck chuck;
    Trajectory<> trajectory;

    TrajectoryGenerator solutionConverter(chuck);
    solutionConverter.fromRotation(Pose<>(1, 1, AngleMath::deg2rad<double>(0)),
        AngleMath::deg2rad<double>(180), 0.1);
    solutionConverter.getTrajectory(trajectory);

    cout << trajectory << endl;
}

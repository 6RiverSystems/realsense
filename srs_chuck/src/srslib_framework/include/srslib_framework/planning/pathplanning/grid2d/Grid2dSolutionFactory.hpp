/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dPosition.hpp>

#include <srslib_framework/localization/map/BaseMap.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>

#include <srslib_framework/robotics/Pose.hpp>

#include <srslib_framework/search/AStar.hpp>

namespace srs {

struct Grid2dSolutionFactory
{
    static Solution<Grid2dSolutionItem>* fromConsecutiveGoals(BaseMap* map,
        Pose<> start, vector<Pose<>> goals);
    static Solution<Grid2dSolutionItem>* fromSingleGoal(BaseMap* map,
        Grid2dPosition& start, Grid2dPosition& goal);
    static Solution<Grid2dSolutionItem>* fromSingleGoal(BaseMap* map,
        Pose<> start, Pose<> goal);
    static Solution<Grid2dSolutionItem>* fromRotation(Pose<> pose,
        double theta0, double thetaf);

private:
    static Solution<Grid2dSolutionItem>* fromSearch(BaseMap* map,
        AStar::SolutionType& intermediateSolution);
};

} // namespace srs

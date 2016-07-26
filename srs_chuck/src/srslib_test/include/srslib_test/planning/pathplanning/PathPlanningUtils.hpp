/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef PATHPLANNINGUTILS_HPP_
#define PATHPLANNINGUTILS_HPP_

#include <gtest/gtest.h>

#include <iostream>
using namespace std;

#include <srslib_framework/localization/Map.hpp>

#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid/PoseAdapter.hpp>

namespace srs {
namespace test {

struct PathPlanningUtils
{
    static Solution<GridSolutionItem>* pose2Solution(Map* map, Pose<> fromPose, Pose<> toPose)
    {
        // Prepare the start position for the search
        Grid2d::LocationType internalStart;
        int startAngle;
        PoseAdapter::pose2Map(fromPose, map, internalStart, startAngle);

        // Prepare the goal position for the search
        Grid2d::LocationType internalGoal;
        int goalAngle;
        PoseAdapter::pose2Map(toPose, map, internalGoal, goalAngle);

        AStar<Grid2d> algorithm(map->getGrid());

        algorithm.search(SearchPosition<Grid2d>(internalStart, startAngle),
            SearchPosition<Grid2d>(internalGoal, goalAngle));

        AStar<Grid2d>::SearchNodeType* solution = algorithm.getSolution();
        return GridSolutionFactory::fromSearch(solution, map);
    }

    static bool checkForNoRotate(Solution<GridSolutionItem>* solution)
    {
        for (auto solutionItem : *solution)
        {
            if (solutionItem.actionType == GridSolutionItem::ROTATE)
            {
                return true;
            }
        }

        return false;
    }
};

} // namespace test
} // namespace srs

#endif // PATHPLANNINGUTILS_HPP_

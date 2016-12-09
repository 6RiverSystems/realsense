/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <gtest/gtest.h>

#include <iostream>
using namespace std;

#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/PoseAdapter.hpp>

namespace srs {
namespace test {

struct PathPlanningUtils
{
    static bool checkForNoRotate(Solution<Grid2dSolutionItem>* solution)
    {
        for (auto solutionItem : *solution)
        {
            if (solutionItem.actionType == Grid2dSolutionItem::ROTATE)
            {
                return true;
            }
        }

        return false;
    }
};

} // namespace test
} // namespace srs

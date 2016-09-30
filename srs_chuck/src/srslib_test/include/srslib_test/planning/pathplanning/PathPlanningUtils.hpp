/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <gtest/gtest.h>

#include <iostream>
using namespace std;

#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/grid/PoseAdapter.hpp>

namespace srs {
namespace test {

struct PathPlanningUtils
{
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

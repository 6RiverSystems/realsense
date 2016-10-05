/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>

namespace srs {

struct GridSolutionUtils
{
    static double getTotalCost(Solution<GridSolutionItem>& solution);
    static vector<Solution<GridSolutionItem>*> splitSolution(Solution<GridSolutionItem>& solution);
};

} // namespace srs

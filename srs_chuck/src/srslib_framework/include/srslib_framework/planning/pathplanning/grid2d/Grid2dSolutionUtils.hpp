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
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>

namespace srs {

struct Grid2dSolutionUtils
{
    static int getTotalCost(Solution<Grid2dSolutionItem>& solution);
    static vector<Solution<Grid2dSolutionItem>*> splitSolution(Solution<Grid2dSolutionItem>& solution);
};

} // namespace srs

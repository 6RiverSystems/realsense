/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef GRIDSOLUTIONFACTORY_HPP_
#define GRIDSOLUTIONFACTORY_HPP_

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/localization/Map.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>

#include <srslib_framework/robotics/Pose.hpp>

#include <srslib_framework/search/SearchNode.hpp>

namespace srs {

struct GridSolutionFactory
{
    static Solution<GridSolutionItem>* fromRotation(Pose<> pose, double theta0, double thetaf);
    static Solution<GridSolutionItem>* fromSearch(SearchNode<Grid2d>* goalNode, Map* map);
};

} // namespace srs

#endif // GRIDSOLUTIONFACTORY_HPP_

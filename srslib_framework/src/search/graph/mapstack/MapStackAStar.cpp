/*
 * (c) Copyright 2017 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srslib_framework/search/graph/mapstack/MapStackAStar.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>

namespace srs {

MapStackAStar::MapStackAStar(MapStack* stack, const SearchParameters& searchParameters) :
    stack_{ stack },
    searchParameters_{ searchParameters }
{
}

MapStackAStar::~MapStackAStar()
{
    delete startNode_;
}

bool MapStackAStar::search(const Position& start, const Position& goal)
{
    MapStackNode::SearchParameters searchParams;

    searchParams.allowUnknown = searchParameters_.allowUnknown;
    searchParams.costMapRatio = searchParameters_.costMapRatio;

    MapStackNode* startNode = MapStackNode::instanceOfStart(stack_, start, searchParams);

    MapStackSingleGoal* goalNode = MapStackSingleGoal::instanceOf(goal);

    return AStar::search(startNode, goalNode);
}

} // namespace srs

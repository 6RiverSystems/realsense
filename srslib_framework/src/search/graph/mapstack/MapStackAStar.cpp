/*
 * (c) Copyright 2017 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srslib_framework/search/graph/mapstack/MapStackAStar.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackAction.hpp>

#include "../../AStar.cpp"

namespace srs {

template class AStar<MapStackNode>;

MapStackAStar::MapStackAStar(MapStack* stack, const MapStackSearchParameters& searchParameters) :
    stack_{ stack },
    searchParameters_{ searchParameters }
{
}

MapStackAStar::~MapStackAStar()
{
    clear();
    startNode_->release();
}

bool MapStackAStar::search(const Position& start, const Position& goal)
{
    MapStackNode* startNode = MapStackNode::instanceOfStart(stack_, start, searchParameters_);

    MapStackSingleGoal* goalNode = MapStackSingleGoal::instanceOf(goal);

    bool ret = AStar::search(startNode, goalNode);

    goalNode->release();

    return ret;
}

void MapStackAStar::releaseNode(MapStackNode* node)
{
	node->release();
}

void MapStackAStar::getExploredNodes(MapStackNode* node, std::vector<MapStackNode*>& nextNodes)
{
    // Find if the next action is allowed
    MapStackNode* neighborNode = MapStackAction::exploreForward(stack_, searchParameters_, node);
    if (neighborNode)
    {
        nextNodes.push_back(neighborNode);
    }

    // Find if the next action is allowed
    neighborNode = MapStackAction::exploreRotation(stack_, searchParameters_, node, MapStackAction::ROTATE_N90, -90);
    if (neighborNode)
    {
        nextNodes.push_back(neighborNode);
    }

    // Find if the next action is allowed
    neighborNode = MapStackAction::exploreRotation(stack_, searchParameters_, node, MapStackAction::ROTATE_P90, +90);
    if (neighborNode)
    {
        nextNodes.push_back(neighborNode);
    }
}

} // namespace srs

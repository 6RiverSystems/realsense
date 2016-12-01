#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapStackNode* MapStackNode::instanceOf(MapStack* stack,
    MapStackNode* parentNode, MapStackAction::ActionEnum parentAction,
    Grid2d::Position position, int g, int h,
    SearchGoal* goal)
{
    return new MapStackNode(stack,
        parentNode, parentAction,
        position,
        g, h, goal);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapStackNode::getExploredNodes(vector<SearchNode*>& exploredNodes)
{
    // Find if the next action is allowed
    MapStackNode* neighborNode = MapStackAction::exploreForward(stack_, this);
    if (neighborNode)
    {
        exploredNodes.push_back(neighborNode);
    }

    // Find if the next action is allowed
    neighborNode = MapStackAction::exploreRotation(stack_, this, MapStackAction::ROTATE_N90, -90);
    if (neighborNode)
    {
        exploredNodes.push_back(neighborNode);
    }

    // Find if the next action is allowed
    neighborNode = MapStackAction::exploreRotation(stack_, this, MapStackAction::ROTATE_P90, +90);
    if (neighborNode)
    {
        exploredNodes.push_back(neighborNode);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapStackNode::release()
{
    delete this;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

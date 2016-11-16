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
void MapStackNode::getNeighbors(vector<SearchNode*>& neighbors)
{
    MapStackAction::ActionResultType actionResult;

    for (MapStackAction::ActionEnum action : MapStackAction::ALLOWED_ACTIONS)
    {
        // Find if the next action is allowed
        if (MapStackAction::execute(stack_, this, action, actionResult))
        {
            // Create a neighbor node with all the relative data
            MapStackNode* neighborNode = MapStackNode::instanceOf(
                stack_,
                this, action,
                actionResult.first, static_cast<int>(actionResult.second), 0,
                goal_);

            // Finally, calculate the heuristic function value
            // from this node to the goal
            neighborNode->h_ = goal_->heuristic(neighborNode);

            // Store it in the
            neighbors.push_back(neighborNode);
        }
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

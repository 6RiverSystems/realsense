#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2dNode::getNeighbors(vector<SearchNode*>& neighbors)
{
    for (Grid2dAction::ActionEnum action : Grid2dAction::ALLOWED_ACTIONS)
    {
        // Find if the next action is allowed
        Grid2dAction::ActionResultType actionResult = Grid2dAction::execute(
            graph_, this, action);

        // If the action is allowed
        if (actionResult.second < Grid2d::COST_MAX)
        {
            // Create a neighbor node with all the relative data
            Grid2dNode* neighborNode = new Grid2dNode(
                graph_,
                this, action,
                actionResult.first, actionResult.second, 0,
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
// Private methods

} // namespace srs

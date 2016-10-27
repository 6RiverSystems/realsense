#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2dNode* Grid2dNode::instanceOf(Grid2d* graph,
    Grid2dNode* parentNode, Grid2dAction::ActionEnum parentAction,
    Grid2d::Position position, int g, int h,
    SearchGoal* goal)
{
    return new Grid2dNode(graph,
        parentNode, parentAction,
        position,
        g, h, goal);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Grid2dNode::getNeighbors(vector<SearchNode*>& neighbors)
{
    Grid2dAction::ActionResultType actionResult;

    for (Grid2dAction::ActionEnum action : Grid2dAction::ALLOWED_ACTIONS)
    {
        // Find if the next action is allowed
        if (Grid2dAction::execute(graph_, this, action, actionResult))
        {
            // Create a neighbor node with all the relative data
            Grid2dNode* neighborNode = Grid2dNode::instanceOf(
                graph_,
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
void Grid2dNode::release()
{
    delete this;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

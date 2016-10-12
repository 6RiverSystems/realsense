#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
array<Grid2dAction::ActionEnum, 5> Grid2dAction::ALLOWED_ACTIONS = {
    FORWARD,
    ROTATE_N90,
    ROTATE_P90,
    ROTATE_180
};

const int Grid2dAction::COMMAND_COSTS[] = {40, 10, 0, 0, 4, 4, 3, 0};

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2dAction::ActionResultType Grid2dAction::execute(Grid2d* graph,
    Grid2dNode* fromNode, ActionEnum action)
{
    switch (action)
    {
        case FORWARD: return addForward(graph, fromNode);
        case BACKWARD: return addBackward(graph, fromNode);
        case ROTATE_N90: return addRotationN90(graph, fromNode);
        case ROTATE_P90: return addRotationP90(graph, fromNode);
        case ROTATE_180: return addRotation180(graph, fromNode);
    }

    return ActionResultType(Grid2dPosition(0, 0), Grid2d::COST_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
unordered_map<int, string> Grid2dAction::ENUM_NAMES = {
    {Grid2dAction::BACKWARD, "BACKWARD"},
    {Grid2dAction::FORWARD, "FORWARD"},
    {Grid2dAction::GOAL, "GOAL"},
    {Grid2dAction::NONE, "NONE"},
    {Grid2dAction::ROTATE_N90, "ROTATE_N90"},
    {Grid2dAction::ROTATE_P90, "ROTATE_P90"},
    {Grid2dAction::ROTATE_180, "ROTATE_180"},
    {Grid2dAction::START, "START"}
};

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2dAction::ActionResultType Grid2dAction::addBackward(Grid2d* graph, Grid2dNode* fromNode)
{
    return ActionResultType(Grid2dPosition(0, 0), Grid2d::COST_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2dAction::ActionResultType Grid2dAction::addForward(Grid2d* graph, Grid2dNode* fromNode)
{
    Grid2d::Location neighbor;
    Grid2dPosition position = fromNode->getPosition();

    Grid2d::Location currentLocation = position.location;
    int currentOrientation = position.orientation;

    if (graph->getNeighbor(currentLocation, position.orientation, neighbor))
    {
        // Start from the total cost of the parent node
        int cost = fromNode->getTotalCost();

        // Add the cost of the command
        cost = BasicMath::noOverflowAdd(cost, COMMAND_COSTS[Grid2dAction::FORWARD]);

        // Add the weight between the parent node and the new location
        cost = BasicMath::noOverflowAdd(cost, graph->getWeight(currentLocation, currentOrientation));

        // Add the cost of the new location
        cost = BasicMath::noOverflowAdd(cost, graph->getCost(neighbor));

        return ActionResultType(Grid2dPosition(neighbor, currentOrientation), cost);
    }

    return ActionResultType(Grid2dPosition(0, 0), Grid2d::COST_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2dAction::ActionResultType Grid2dAction::addRotationN90(Grid2d* graph, Grid2dNode* fromNode)
{
    return ActionResultType(Grid2dPosition(0, 0), Grid2d::COST_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2dAction::ActionResultType Grid2dAction::addRotationP90(Grid2d* graph, Grid2dNode* fromNode)
{
    return ActionResultType(Grid2dPosition(0, 0), Grid2d::COST_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2dAction::ActionResultType Grid2dAction::addRotation180(Grid2d* graph, Grid2dNode* fromNode)
{
    return ActionResultType(Grid2dPosition(0, 0), Grid2d::COST_MAX);
}

} // namespace srs

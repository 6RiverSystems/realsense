#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
array<Grid2dAction::ActionEnum, 3> Grid2dAction::ALLOWED_ACTIONS = {
    FORWARD,
    ROTATE_N90,
    ROTATE_P90
};

const int Grid2dAction::COMMAND_COSTS[] = {
    40, // BACKWARD
    1, // FORWARD
    0, // NONE
    2, // ROTATE_N90
    2, // ROTATE_P90
    3, // ROTATE_180
    0 // START
};

////////////////////////////////////////////////////////////////////////////////////////////////////
bool Grid2dAction::execute(Grid2d* graph, Grid2dNode* fromNode, ActionEnum action,
    ActionResultType& result)
{
    switch (action)
    {
        case FORWARD: return addForward(graph, fromNode, result);
        case BACKWARD: return addBackward(graph, fromNode, result);
        case ROTATE_N90: return addRotation(graph, fromNode, ROTATE_N90, -90, result);
        case ROTATE_P90: return addRotation(graph, fromNode, ROTATE_P90, +90, result);
        case ROTATE_180: return addRotation(graph, fromNode, ROTATE_180, +180, result);
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
unordered_map<int, string> Grid2dAction::ENUM_NAMES = {
    {Grid2dAction::BACKWARD, "BACKWARD"},
    {Grid2dAction::FORWARD, "FORWARD"},
    {Grid2dAction::NONE, "NONE"},
    {Grid2dAction::ROTATE_N90, "ROTATE_N90"},
    {Grid2dAction::ROTATE_P90, "ROTATE_P90"},
    {Grid2dAction::ROTATE_180, "ROTATE_180"},
    {Grid2dAction::START, "START"}
};

////////////////////////////////////////////////////////////////////////////////////////////////////
bool Grid2dAction::addBackward(Grid2d* graph, Grid2dNode* fromNode,
    ActionResultType& result)
{
    Grid2d::Location neighbor;
    Grid2dPosition position = fromNode->getPosition();

    Grid2d::Location currentLocation = position.location;
    int currentOrientation = position.orientation;
    int directionMovement = AngleMath::normalizeDeg<int>(currentOrientation + 180);

    if (graph->getNeighbor(currentLocation, directionMovement, neighbor))
    {
        // Start from the total cost of the parent node
        int cost = fromNode->getTotalCost();

        // Add the cost of the command
        cost += COMMAND_COSTS[Grid2dAction::BACKWARD];

        // Add the weight between the parent node and the new location
        cost += graph->getWeight(currentLocation, directionMovement);

        // Add the cost of the new location
        cost += cost, graph->getAggregate(neighbor);

        result = ActionResultType(Grid2dPosition(neighbor, currentOrientation), cost);
        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool Grid2dAction::addForward(Grid2d* graph, Grid2dNode* fromNode,
    ActionResultType& result)
{
    Grid2d::Location neighbor;
    Grid2dPosition position = fromNode->getPosition();

    Grid2d::Location currentLocation = position.location;
    int currentOrientation = position.orientation;

    if (graph->getNeighbor(currentLocation, currentOrientation, neighbor))
    {
        // Start from the total cost of the parent node
        int cost = fromNode->getG();

        // Add the cost of the command
        cost += COMMAND_COSTS[Grid2dAction::FORWARD];

        // Add the weight between the parent node and the new location
        cost += graph->getWeight(currentLocation, currentOrientation);

        // Add the cost of the new location
        cost += graph->getAggregate(neighbor);

        result = ActionResultType(Grid2dPosition(neighbor, currentOrientation), cost);
        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool Grid2dAction::addRotation(Grid2d* graph, Grid2dNode* fromNode, ActionEnum action, int angle,
    ActionResultType& result)
{
    Grid2dPosition position = fromNode->getPosition();

    Grid2d::Location currentLocation = position.location;
    int currentOrientation = position.orientation;
    int newOrientation = AngleMath::normalizeDeg<int>(currentOrientation + angle);

    // Start from the total cost of the parent node
    int cost = fromNode->getTotalCost();

    // Add the cost of the command
    cost += COMMAND_COSTS[action];

    result = ActionResultType(Grid2dPosition(currentLocation, newOrientation), cost);
    return true;
}

} // namespace srs

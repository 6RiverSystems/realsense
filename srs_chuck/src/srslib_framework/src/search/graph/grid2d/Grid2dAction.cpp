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
    Grid2d::Position fromPosition = fromNode->getPosition();

    int directionMovement = AngleMath::normalizeDeg<int>(fromPosition.orientation + 180);
    Grid2d::Position motion = Grid2d::Position(fromPosition.x, fromPosition.y, directionMovement);

    Grid2d::Position neighbor;
    if (graph->getNeighbor(motion, neighbor))
    {
        // Calculate the motion cost
        int motionCost = COMMAND_COSTS[Grid2dAction::BACKWARD];

        int localCost = graph->getWeight(fromPosition);
        if (isCostAvailable(localCost))
        {
            motionCost += localCost;

            localCost = graph->getPayload(neighbor);
            if (isCostAvailable(localCost))
            {
                motionCost += localCost;

                result = ActionResultType(neighbor, fromNode->getG() + motionCost);
                return true;
            }
        }
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool Grid2dAction::addForward(Grid2d* graph, Grid2dNode* fromNode,
    ActionResultType& result)
{
    Grid2d::Position fromPosition = fromNode->getPosition();

    Grid2d::Position neighbor;
    if (graph->getNeighbor(fromPosition, neighbor))
    {
        // Calculate the motion cost
        int motionCost = COMMAND_COSTS[Grid2dAction::FORWARD];

        int localCost = graph->getWeight(fromPosition);
        if (isCostAvailable(localCost))
        {
            motionCost += localCost;

            localCost = graph->getPayload(neighbor);
            if (isCostAvailable(localCost))
            {
                motionCost += localCost;

                result = ActionResultType(neighbor, fromNode->getG() + motionCost);
                return true;
            }
        }
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool Grid2dAction::addRotation(Grid2d* graph, Grid2dNode* fromNode, ActionEnum action, int angle,
    ActionResultType& result)
{
    Grid2d::Position fromPosition = fromNode->getPosition();

    int newOrientation = AngleMath::normalizeDeg<int>(fromPosition.orientation + angle);
    Grid2d::Position motion = Grid2d::Position(fromPosition.x, fromPosition.y, newOrientation);

    result = ActionResultType(motion, fromNode->getG() + COMMAND_COSTS[action]);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool Grid2dAction::isCostAvailable(int& cost)
{
    if (cost == Grid2d::PAYLOAD_NO_INFORMATION)
    {
        cost = 0;
    }
    return cost < Grid2d::PAYLOAD_MAX;
}

} // namespace srs

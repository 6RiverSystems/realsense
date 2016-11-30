#include <srslib_framework/search/graph/mapstack/MapStackAction.hpp>

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
array<MapStackAction::ActionEnum, 3> MapStackAction::ALLOWED_ACTIONS = {
    FORWARD,
    ROTATE_N90,
    ROTATE_P90
};

const int MapStackAction::COMMAND_COSTS[] = {
    0, // BACKWARD
    0, // FORWARD
    0, // NONE
    0, // ROTATE_N90
    0, // ROTATE_P90
    0, // ROTATE_180
    0 // START
};

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapStackAction::execute(MapStack* stack, MapStackNode* fromNode, ActionEnum action,
    ActionResultType& result)
{
    switch (action)
    {
        case FORWARD: return addForward(stack, fromNode, result);
        case BACKWARD: return addBackward(stack, fromNode, result);
        case ROTATE_N90: return addRotation(stack, fromNode, ROTATE_N90, -90, result);
        case ROTATE_P90: return addRotation(stack, fromNode, ROTATE_P90, +90, result);
        case ROTATE_180: return addRotation(stack, fromNode, ROTATE_180, +180, result);
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
unordered_map<int, string> MapStackAction::ENUM_NAMES = {
    {MapStackAction::BACKWARD, "BACKWARD"},
    {MapStackAction::FORWARD, "FORWARD"},
    {MapStackAction::NONE, "NONE"},
    {MapStackAction::ROTATE_N90, "ROTATE_N90"},
    {MapStackAction::ROTATE_P90, "ROTATE_P90"},
    {MapStackAction::ROTATE_180, "ROTATE_180"},
    {MapStackAction::START, "START"}
};

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapStackAction::addBackward(MapStack* stack, MapStackNode* fromNode,
    ActionResultType& result)
{
    Grid2d::Position fromPosition = fromNode->getPosition();

    int directionMovement = AngleMath::normalizeDeg<int>(fromPosition.orientation + 180);
    Grid2d::Position motion = Grid2d::Position(fromPosition.x, fromPosition.y, directionMovement);

    Grid2d::Position neighbor;
    if (stack->getNeighbor(motion, neighbor))
    {
        // Calculate the motion cost
        int motionCost = COMMAND_COSTS[MapStackAction::BACKWARD];

        int localCost = stack->getWeight(fromPosition);
        if (isCostAvailable(localCost))
        {
            motionCost += localCost;

            localCost = stack->getTotalCost(neighbor);
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
bool MapStackAction::addForward(MapStack* stack, MapStackNode* fromNode,
    ActionResultType& result)
{
    Grid2d::Position fromPosition = fromNode->getPosition();

    Grid2d::Position neighbor;
    if (stack->getNeighbor(fromPosition, neighbor))
    {
        // Calculate the motion cost
        int motionCost = COMMAND_COSTS[MapStackAction::FORWARD];

        int localCost = stack->getWeight(fromPosition);
        if (isCostAvailable(localCost))
        {
            motionCost += localCost;

            localCost = stack->getTotalCost(neighbor);
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
bool MapStackAction::addRotation(MapStack* stack, MapStackNode* fromNode, ActionEnum action, int angle,
    ActionResultType& result)
{
    Grid2d::Position fromPosition = fromNode->getPosition();

    int newOrientation = AngleMath::normalizeDeg<int>(fromPosition.orientation + angle);
    Grid2d::Position motion = Grid2d::Position(fromPosition.x, fromPosition.y, newOrientation);

    result = ActionResultType(motion, fromNode->getG() + COMMAND_COSTS[action]);
    return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapStackAction::isCostAvailable(int& cost)
{
    if (cost == Grid2d::PAYLOAD_NO_INFORMATION)
    {
        cost = 0;
    }
    return cost < Grid2d::PAYLOAD_MAX;
}

} // namespace srs

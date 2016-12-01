#include <srslib_framework/search/graph/mapstack/MapStackAction.hpp>

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

const int MapStackAction::COMMAND_COSTS[] = {
    1, // BACKWARD
    1, // FORWARD
    0, // NONE
    1, // ROTATE_N90
    1, // ROTATE_P90
    1, // ROTATE_180
    0 // START
};

////////////////////////////////////////////////////////////////////////////////////////////////////
MapStackNode* MapStackAction::exploreBackward(MapStack* stack, MapStackNode* fromNode)
{
    Grid2d::Position fromPosition = fromNode->getPosition();

    int directionMovement = AngleMath::normalizeDeg<int>(fromPosition.orientation + 180);
    Grid2d::Position motion = Grid2d::Position(fromPosition.x, fromPosition.y, directionMovement);

    Grid2d::Position neighbor;
    if (stack->getNeighbor(motion, neighbor))
    {
        // Calculate the motion cost
        int motionCost = COMMAND_COSTS[MapStackAction::BACKWARD];
        motionCost += stack->getWeight(fromPosition);
        motionCost += stack->getTotalCost(neighbor, false);

        if (motionCost < Grid2d::PAYLOAD_MAX)
        {
            // Create a neighbor node with all the relative data
            MapStackNode* neighborNode = MapStackNode::instanceOf(
                stack,
                fromNode, MapStackAction::BACKWARD,
                motion,
                fromNode->getG() + motionCost, 0,
                nullptr);

            // Finally, calculate the heuristic function value
            // from this node to the goal
            neighborNode->setGoal(fromNode->getGoal());

            return neighborNode;
        }
    }

    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
MapStackNode* MapStackAction::exploreForward(MapStack* stack, MapStackNode* fromNode)
{
    Grid2d::Position fromPosition = fromNode->getPosition();

    Grid2d::Position motion;
    if (stack->getNeighbor(fromPosition, motion))
    {
        // Calculate the motion cost
        int motionCost = COMMAND_COSTS[MapStackAction::FORWARD];
        motionCost += stack->getWeight(fromPosition);
        motionCost += stack->getTotalCost(motion, false);

        if (motionCost < Grid2d::PAYLOAD_MAX)
        {
            // Create a neighbor node with all the relative data
            MapStackNode* neighborNode = MapStackNode::instanceOf(
                stack,
                fromNode, MapStackAction::FORWARD,
                motion,
                fromNode->getG() + motionCost, 0,
                nullptr);

            // Finally, calculate the heuristic function value
            // from this node to the goal
            neighborNode->setGoal(fromNode->getGoal());

            return neighborNode;
        }
    }

    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
MapStackNode* MapStackAction::exploreRotation(MapStack* stack, MapStackNode* fromNode,
    ActionEnum action, int angle)
{
    Grid2d::Position fromPosition = fromNode->getPosition();

    int newOrientation = AngleMath::normalizeDeg<int>(fromPosition.orientation + angle);
    Grid2d::Position motion = Grid2d::Position(fromPosition.x, fromPosition.y, newOrientation);

    // Calculate the motion cost
    int motionCost = COMMAND_COSTS[action];
    motionCost += stack->getTotalCost(motion, false);

    if (motionCost < Grid2d::PAYLOAD_MAX)
    {
        // Create a neighbor node with all the relative data
        MapStackNode* neighborNode = MapStackNode::instanceOf(
            stack,
            fromNode, MapStackAction::ROTATE_N90,
            motion,
            fromNode->getG() + motionCost, 0,
            nullptr);

        // Finally, calculate the heuristic function value
        // from this node to the goal
        neighborNode->setGoal(fromNode->getGoal());

        return neighborNode;
    }

    return nullptr;
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

} // namespace srs

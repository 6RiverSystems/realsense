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
    Position fromPosition = fromNode->getPosition();

    int directionMovement = AngleMath::normalizeDeg<int>(fromPosition.orientation + 180);
    Position motion = Position(fromPosition.location, directionMovement);

    Position neighbor;
    if (stack->getNeighbor(motion, neighbor))
    {
        MapStackNode::SearchParameters searchParameters = fromNode->getSearchParameters();

        // Calculate the motion cost
        int motionCost = COMMAND_COSTS[MapStackAction::BACKWARD];
        motionCost += stack->getWeight(fromPosition);
        motionCost += stack->getTotalCost(neighbor,
            searchParameters.allowUnknown,
            searchParameters.costMapRatio);

        if (motionCost < PRUNING_THRESHOLD)
        {
            // Create a neighbor node with all the relative data
            MapStackNode* neighborNode = MapStackNode::instanceOf(
                stack,
                fromNode, MapStackAction::BACKWARD,
                motion,
                fromNode->getLocalCost() + motionCost, 0,
                nullptr,
                searchParameters);

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
    Position fromPosition = fromNode->getPosition();

    Position motion;
    if (stack->getNeighbor(fromPosition, motion))
    {
        MapStackNode::SearchParameters searchParameters = fromNode->getSearchParameters();

        // Calculate the motion cost
        int motionCost = COMMAND_COSTS[MapStackAction::FORWARD];
        motionCost += stack->getWeight(fromPosition);
        motionCost += stack->getTotalCost(motion,
            searchParameters.allowUnknown,
            searchParameters.costMapRatio);

        if (motionCost < PRUNING_THRESHOLD)
        {
            // Create a neighbor node with all the relative data
            MapStackNode* neighborNode = MapStackNode::instanceOf(
                stack,
                fromNode, MapStackAction::FORWARD,
                motion,
                fromNode->getLocalCost() + motionCost, 0,
                nullptr,
                searchParameters);

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
    Position fromPosition = fromNode->getPosition();

    int newOrientation = AngleMath::normalizeDeg<int>(fromPosition.orientation + angle);
    Position motion = Position(fromPosition.location, newOrientation);

    MapStackNode::SearchParameters searchParameters = fromNode->getSearchParameters();

    // Calculate the motion cost
    int motionCost = COMMAND_COSTS[action];
    motionCost += stack->getTotalCost(motion,
        searchParameters.allowUnknown,
        searchParameters.costMapRatio);

    if (motionCost < PRUNING_THRESHOLD)
    {
        // Create a neighbor node with all the relative data
        MapStackNode* neighborNode = MapStackNode::instanceOf(
            stack,
            fromNode, action,
            motion,
            fromNode->getLocalCost() + motionCost, 0,
            nullptr,
            searchParameters);

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

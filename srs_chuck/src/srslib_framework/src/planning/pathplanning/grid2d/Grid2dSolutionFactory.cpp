#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>

#include <srslib_framework/planning/pathplanning/grid2d/UnexpectedSearchActionException.hpp>
#include <srslib_framework/search/SearchNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackAction.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromConsecutiveGoals(MapStack* stack,
    Pose<> start, vector<Pose<>> goals,
    AStar::ConfigParameters configParameters,
    MapStackNode::SearchParameters searchParameters)
{
    Solution<Grid2dSolutionItem>* globalSolution =
        Solution<Grid2dSolutionItem>::instanceOfValidEmpty();

    unsigned int exploredNodes = 0;

    Pose<> intermediateStart = start;
    for (Pose<> goal : goals)
    {
        Solution<Grid2dSolutionItem>* localSolution = fromSingleGoal(stack, intermediateStart,
            goal, configParameters, searchParameters);
        if (!localSolution->isValid())
        {
            globalSolution->setValid(false);
            break;
        }

        exploredNodes += localSolution->getExploredNodes();

        globalSolution->append(localSolution);
        intermediateStart = goal;
    }

    globalSolution->setExploredNodes(exploredNodes);

    return globalSolution;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromSingleGoal(MapStack* stack,
    Position& start, Position& goal,
    AStar::ConfigParameters configParameters,
    MapStackNode::SearchParameters searchParameters)
{
    if (!stack)
    {
        return nullptr;
    }

    MapStackNode* startNode = MapStackNode::instanceOfStart(stack, start, searchParameters);
    MapStackSingleGoal* goalNode = MapStackSingleGoal::instanceOf(goal);

    AStar algorithm;

    if (algorithm.search(startNode, goalNode, configParameters))
    {
        Plan plan;
        algorithm.getPlan(plan);

        return fromSearch(stack->getLogicalMap(), plan);
    }

    return Solution<Grid2dSolutionItem>::instanceOfInvalidEmpty();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromSingleGoal(MapStack* stack,
    Pose<> fromPose, Pose<> toPose,
    AStar::ConfigParameters configParameters,
    MapStackNode::SearchParameters searchParameters)
{
    if (!stack)
    {
        return nullptr;
    }

    // Prepare the start position for the search
    Position start = pose2Map(stack->getLogicalMap(), fromPose);

    // Prepare the goal position for the search
    Position goal = pose2Map(stack->getLogicalMap(), toPose);

    return fromSingleGoal(stack, start, goal, configParameters, searchParameters);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromRotation(
    Pose<> pose, double theta0, double thetaf)
{
    Grid2dSolutionItem solutionItem;

    solutionItem.actionType = Grid2dSolutionItem::ROTATE;

    solutionItem.fromPose = pose;
    solutionItem.toPose = pose;

    solutionItem.fromPose.theta = AngleMath::normalizeRad(theta0);
    solutionItem.toPose.theta = AngleMath::normalizeRad(thetaf);

    return Solution<Grid2dSolutionItem>::instanceOfValid(solutionItem);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromSearch(BaseMap* map, Plan& plan)
{
    if (!map)
    {
        return nullptr;
    }

    Solution<Grid2dSolutionItem>* solution = Solution<Grid2dSolutionItem>::instanceOfValidEmpty();
    solution->setExploredNodes(plan.getClosedNodesCount() + plan.getOpenNodesCount());
    solution->setTotalCost(plan.getTotalCost());

    Grid2dSolutionItem solutionItem;
    Pose<> fromPose;
    Pose<> toPose;

    double fromX = 0;
    double fromY = 0;
    double fromTheta = 0;

    double toX = 0;
    double toY = 0;
    double toTheta = 0;

    Plan::const_iterator fromCursor = plan.begin();
    Plan::const_iterator toCursor = next(fromCursor, 1);

    while (toCursor != plan.end())
    {
        MapStackNode* fromNode = reinterpret_cast<MapStackNode*>(*fromCursor);
        MapStackNode* toNode = reinterpret_cast<MapStackNode*>(*toCursor);

        map->transformCells2M(
            fromNode->getPosition().location.x, fromNode->getPosition().location.y,
            fromX, fromY);

        fromTheta = AngleMath::deg2Rad<double>(fromNode->getPosition().orientation);
        fromPose = Pose<>(fromX, fromY, AngleMath::normalizeRad<double>(fromTheta));

        map->transformCells2M(
            toNode->getPosition().location.x, toNode->getPosition().location.y,
            toX, toY);

        toTheta = AngleMath::deg2Rad<double>(toNode->getPosition().orientation);
        toPose = Pose<>(toX, toY, AngleMath::normalizeRad<double>(toTheta));

        switch (toNode->getParentAction())
        {
            case MapStackAction::FORWARD:
            case MapStackAction::BACKWARD:
                solutionItem.actionType = Grid2dSolutionItem::MOVE;
                break;

            case MapStackAction::ROTATE_N90:
            case MapStackAction::ROTATE_P90:
            case MapStackAction::ROTATE_180:
                solutionItem.actionType = Grid2dSolutionItem::ROTATE;
                break;

            default:
                // It should never see a NONE or START
                throw UnexpectedSearchActionException(plan);
        }

        solutionItem.fromPose = fromPose;
        solutionItem.toPose = toPose;
        solutionItem.cost = toNode->getLocalCost();

        solution->push_back(solutionItem);

        fromCursor++;
        toCursor++;
    }

    return solution;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Position Grid2dSolutionFactory::pose2Map(BaseMap* map, Pose<> pose)
{
    unsigned int r = 0;
    unsigned int c = 0;
    map->transformM2Cells(pose, c, r);

    int orientation = static_cast<int>(AngleMath::normalizeRad2Deg90(pose.theta));
    return Position(c, r, orientation);
}

} // namespace srs

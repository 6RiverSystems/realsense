#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>

#include <srslib_framework/planning/pathplanning/grid2d/exception/UnexpectedSearchActionException.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/PoseAdapter.hpp>
#include <srslib_framework/search/SearchNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dAction.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromConsecutiveGoals(BaseMap* map,
    Pose<> start, vector<Pose<>> goals)
{
    Solution<Grid2dSolutionItem>* globalSolution = new Solution<Grid2dSolutionItem>();

    Pose<> intermediateStart = start;
    for (Pose<> goal : goals)
    {
        Solution<Grid2dSolutionItem>* localSolution = fromSingleGoal(map, intermediateStart, goal);
        if (localSolution->empty())
        {
            break;
        }

        globalSolution->append(localSolution);
        intermediateStart = goal;
    }

    return globalSolution;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromSingleGoal(BaseMap* map,
    Grid2d::Position& start, Grid2d::Position& goal)
{
    if (!map)
    {
        return nullptr;
    }

    Grid2d* grid = map->getGrid();

    Grid2dNode* startNode = Grid2dNode::instanceOfStart(grid, start);
    Grid2dSingleGoal* goalNode = Grid2dSingleGoal::instanceOf(goal);

    AStar algorithm;

    algorithm.search(startNode, goalNode);

    AStar::SolutionType solution;
    algorithm.getSolution(solution);

    return fromSearch(map, solution);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromSingleGoal(BaseMap* map,
    Pose<> fromPose, Pose<> toPose)
{
    if (!map)
    {
        return nullptr;
    }

    // Prepare the start position for the search
    Grid2d::Position start = PoseAdapter::pose2Map(fromPose, map);

    // Prepare the goal position for the search
    Grid2d::Position goal = PoseAdapter::pose2Map(toPose, map);

    return fromSingleGoal(map, start, goal);
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

    return new Solution<Grid2dSolutionItem>(solutionItem);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromSearch(BaseMap* map,
    AStar::SolutionType& intermediateSolution)
{
    if (!map)
    {
        return nullptr;
    }

    Solution<Grid2dSolutionItem>* result = new Solution<Grid2dSolutionItem>();

    // If no path was found in the previous search,
    // exit immediately and return an empty solution
    if (intermediateSolution.empty())
    {
        return result;
    }

    Grid2dSolutionItem solutionItem;
    Pose<> fromPose;
    Pose<> toPose;

    double fromX = 0;
    double fromY = 0;
    double fromTheta = 0;

    double toX = 0;
    double toY = 0;
    double toTheta = 0;

    AStar::SolutionType::const_iterator fromCursor = intermediateSolution.begin();
    AStar::SolutionType::const_iterator toCursor = next(fromCursor, 1);

    while (toCursor != intermediateSolution.end())
    {
        Grid2dNode* fromNode = reinterpret_cast<Grid2dNode*>(*fromCursor);
        Grid2dNode* toNode = reinterpret_cast<Grid2dNode*>(*toCursor);

        map->transformCells2M(
            fromNode->getPosition().x, fromNode->getPosition().y,
            fromX, fromY);

        fromTheta = AngleMath::deg2Rad<double>(fromNode->getPosition().orientation);
        fromPose = Pose<>(fromX, fromY, AngleMath::normalizeRad<double>(fromTheta));

        map->transformCells2M(
            toNode->getPosition().x, toNode->getPosition().y,
            toX, toY);

        toTheta = AngleMath::deg2Rad<double>(toNode->getPosition().orientation);
        toPose = Pose<>(toX, toY, AngleMath::normalizeRad<double>(toTheta));

        switch (toNode->getParentAction())
        {
            case Grid2dAction::FORWARD:
            case Grid2dAction::BACKWARD:
                solutionItem.actionType = Grid2dSolutionItem::MOVE;
                break;

            case Grid2dAction::ROTATE_N90:
            case Grid2dAction::ROTATE_P90:
            case Grid2dAction::ROTATE_180:
                solutionItem.actionType = Grid2dSolutionItem::ROTATE;
                break;

            default:
                // It should never see a NONE or START
                throw UnexpectedSearchActionException(intermediateSolution);
        }

        solutionItem.fromPose = fromPose;
        solutionItem.toPose = toPose;
        solutionItem.cost = toNode->getG();

        result->push_back(solutionItem);

        fromCursor++;
        toCursor++;
    }

    return result;
}

} // namespace srs

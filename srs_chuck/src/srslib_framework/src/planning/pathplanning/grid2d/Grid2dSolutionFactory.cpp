#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>

#include <srslib_framework/planning/pathplanning/grid2d/UnexpectedSearchActionException.hpp>
#include <srslib_framework/search/SearchNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dNode.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dAction.hpp>
#include <srslib_framework/search/graph/grid2d/Grid2dSingleGoal.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackAction.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackSingleGoal.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromConsecutiveGoals(BaseMap* map,
    Pose<> start, vector<Pose<>> goals)
{
    Solution<Grid2dSolutionItem>* globalSolution =
        Solution<Grid2dSolutionItem>::instanceOfValidEmpty();

    Pose<> intermediateStart = start;
    for (Pose<> goal : goals)
    {
        Solution<Grid2dSolutionItem>* localSolution = fromSingleGoal(map, intermediateStart, goal);
        if (!localSolution->isValid())
        {
            globalSolution->setValid(false);
            break;
        }

        globalSolution->append(localSolution);
        intermediateStart = goal;
    }

    return globalSolution;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromConsecutiveGoals(MapStack* stack,
    Pose<> start, vector<Pose<>> goals)
{
    Solution<Grid2dSolutionItem>* globalSolution =
        Solution<Grid2dSolutionItem>::instanceOfValidEmpty();

    Pose<> intermediateStart = start;
    for (Pose<> goal : goals)
    {
        Solution<Grid2dSolutionItem>* localSolution = fromSingleGoal(stack, intermediateStart, goal);
        if (!localSolution->isValid())
        {
            globalSolution->setValid(false);
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

    if (algorithm.search(startNode, goalNode))
    {
        AStar::SolutionType searchSolution;
        algorithm.getSolution(searchSolution);

        return fromSearch(map, searchSolution);
    }

    return Solution<Grid2dSolutionItem>::instanceOfInvalidEmpty();
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
    Grid2d::Position start = pose2Map(map, fromPose);

    // Prepare the goal position for the search
    Grid2d::Position goal = pose2Map(map, toPose);

    return fromSingleGoal(map, start, goal);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromSingleGoal(MapStack* stack,
    Grid2d::Position& start, Grid2d::Position& goal)
{
    if (!stack)
    {
        return nullptr;
    }

    MapStackNode* startNode = MapStackNode::instanceOfStart(stack, start);
    MapStackSingleGoal* goalNode = MapStackSingleGoal::instanceOf(goal);

    AStar algorithm;

    if (algorithm.search(startNode, goalNode))
    {
        AStar::SolutionType searchSolution;
        algorithm.getSolution(searchSolution);

        return fromSearch(stack->getLogicalMap(), searchSolution);
    }

    return Solution<Grid2dSolutionItem>::instanceOfInvalidEmpty();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromSingleGoal(MapStack* stack,
    Pose<> fromPose, Pose<> toPose)
{
    if (!stack)
    {
        return nullptr;
    }

    // Prepare the start position for the search
    Grid2d::Position start = pose2Map(stack->getLogicalMap(), fromPose);

    // Prepare the goal position for the search
    Grid2d::Position goal = pose2Map(stack->getLogicalMap(), toPose);

    return fromSingleGoal(stack, start, goal);
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
Solution<Grid2dSolutionItem>* Grid2dSolutionFactory::fromSearch(BaseMap* map,
    AStar::SolutionType& searchSolution)
{
    if (!map)
    {
        return nullptr;
    }

    Solution<Grid2dSolutionItem>* solution = Solution<Grid2dSolutionItem>::instanceOfValidEmpty();

    Grid2dSolutionItem solutionItem;
    Pose<> fromPose;
    Pose<> toPose;

    double fromX = 0;
    double fromY = 0;
    double fromTheta = 0;

    double toX = 0;
    double toY = 0;
    double toTheta = 0;

    AStar::SolutionType::const_iterator fromCursor = searchSolution.begin();
    AStar::SolutionType::const_iterator toCursor = next(fromCursor, 1);

    while (toCursor != searchSolution.end())
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
                throw UnexpectedSearchActionException(searchSolution);
        }

        solutionItem.fromPose = fromPose;
        solutionItem.toPose = toPose;
        solutionItem.cost = toNode->getG();

        solution->push_back(solutionItem);

        fromCursor++;
        toCursor++;
    }

    return solution;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2d::Position Grid2dSolutionFactory::pose2Map(BaseMap* map, Pose<> pose)
{
    unsigned int r = 0;
    unsigned int c = 0;
    map->transformM2Cells(pose, c, r);

    int orientation = static_cast<int>(AngleMath::normalizeRad2Deg90(pose.theta));
    return Grid2d::Position(c, r, orientation);
}

} // namespace srs

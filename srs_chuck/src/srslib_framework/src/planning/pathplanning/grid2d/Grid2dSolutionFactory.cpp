#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>

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
    Pose<> fromPose, Pose<> toPose)
{
    if (!map)
    {
        return nullptr;
    }

    // Prepare the start position for the search
    Grid2d::Location internalStart;
    int startAngle;
    PoseAdapter::pose2Map(fromPose, map, internalStart, startAngle);

    // Prepare the goal position for the search
    Grid2d::Location internalGoal;
    int goalAngle;
    PoseAdapter::pose2Map(toPose, map, internalGoal, goalAngle);

    Grid2d* grid = map->getGrid();

    Grid2dPosition startPosition(internalStart, startAngle);
    Grid2dNode* start = Grid2dNode::instanceOfStart(grid, Grid2dPosition(internalStart, startAngle));

    Grid2dPosition goalPosition(internalGoal, goalAngle);
    Grid2dSingleGoal* goal = Grid2dSingleGoal::instanceOf(Grid2dPosition(internalGoal, goalAngle));

    AStar algorithm;

    algorithm.search(start, goal);

    AStar::SolutionType solution;
    algorithm.getSolution(solution);

    return fromSearch(map, solution);
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
    AStar::SolutionType::const_iterator toCursor = ++fromCursor;

    bool insertNode;

    while (toCursor != intermediateSolution.end())
    {
        insertNode = true;

        Grid2dNode* fromNode = reinterpret_cast<Grid2dNode*>(*fromCursor);
        Grid2dNode* toNode = reinterpret_cast<Grid2dNode*>(*toCursor);

        map->transformCells2Mm(
            fromNode->getPosition().location.x, fromNode->getPosition().location.y,
            toX, toY);

        fromTheta = AngleMath::deg2Rad<double>(fromNode->getPosition().orientation);
        fromPose = Pose<>(fromX, fromY, AngleMath::normalizeRad<double>(fromTheta));

        map->transformCells2Mm(
            toNode->getPosition().location.x, toNode->getPosition().location.y,
            toX, toY);

        toTheta = AngleMath::deg2Rad<double>(toNode->getPosition().orientation);
        toPose = Pose<>(toX, toY, AngleMath::normalizeRad<double>(toTheta));

        switch (toNode->getParentAction())
        {
            case Grid2dAction::START:
                insertNode = false;
                break;

            case Grid2dAction::FORWARD:
                solutionItem.actionType = Grid2dSolutionItem::MOVE;
                break;

            case Grid2dAction::BACKWARD:
                solutionItem.actionType = Grid2dSolutionItem::MOVE;
                break;

            case Grid2dAction::ROTATE_N90:
                solutionItem.actionType = Grid2dSolutionItem::ROTATE;
                fromTheta = AngleMath::normalizeRad<double>(
                    toTheta + AngleMath::deg2Rad<double>(90));
                fromPose = Pose<>(toX, toY, fromTheta);
                break;

            case Grid2dAction::ROTATE_P90:
                solutionItem.actionType = Grid2dSolutionItem::ROTATE;
                fromTheta = AngleMath::normalizeRad<double>(
                    toTheta - AngleMath::deg2Rad<double>(90));
                fromPose = Pose<>(toX, toY, fromTheta);
                break;

            case Grid2dAction::ROTATE_180:
                solutionItem.actionType = Grid2dSolutionItem::ROTATE;
                fromTheta = AngleMath::normalizeRad<double>(
                    toTheta - AngleMath::deg2Rad<double>(180));
                fromPose = Pose<>(toX, toY, fromTheta);
                break;

            default:
                // TODO: Add exception
                // It should never see a NONE
                throw;
        }

        //
        //        solutionItem.fromPose = fromPose;
        //        solutionItem.toPose = toPose;
        //        solutionItem.cost = toCursor->action->getTotalCost();
        //
        //        if (insertNode)
        //        {
        //            result->insert(result->begin(), solutionItem);
        //        }
        //
        //        toCursor = toCursor->parent;
        //        if (fromCursor)
        //        {
        //            fromCursor = fromCursor->parent;
        //        }
        //    }
        //
        //    return result;
        //}
    }

    return nullptr;
}

} // namespace srs

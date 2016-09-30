#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>

#include <srslib_framework/planning/pathplanning/grid/PoseAdapter.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<GridSolutionItem>* GridSolutionFactory::fromConsecutiveGoals(BaseMap* map,
    Pose<> start, vector<Pose<>> goals)
{
    Solution<GridSolutionItem>* globalSolution = new Solution<GridSolutionItem>();

    Pose<> intermediateStart = start;
    for (Pose<> goal : goals)
    {
        Solution<GridSolutionItem>* localSolution = GridSolutionFactory::fromGoal(map,
            intermediateStart, goal);
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
Solution<GridSolutionItem>* GridSolutionFactory::fromGoal(BaseMap* map,
    Pose<> fromPose, Pose<> toPose)
{
    if (!map)
    {
        return nullptr;
    }

    // Prepare the start position for the search
    Grid2d::LocationType internalStart;
    int startAngle;
    PoseAdapter::pose2Map(fromPose, map, internalStart, startAngle);

    // Prepare the goal position for the search
    Grid2d::LocationType internalGoal;
    int goalAngle;
    PoseAdapter::pose2Map(toPose, map, internalGoal, goalAngle);

    AStar<Grid2d> algorithm(map->getGrid());

    algorithm.search(SearchPosition<Grid2d>(internalStart, startAngle),
        SearchPosition<Grid2d>(internalGoal, goalAngle));

    AStar<Grid2d>::SearchNodeType* solution = algorithm.getSolution();
    return GridSolutionFactory::fromSearch(map, solution);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<GridSolutionItem>* GridSolutionFactory::fromRotation(
    Pose<> pose, double theta0, double thetaf)
{
    GridSolutionItem solutionItem;

    solutionItem.actionType = GridSolutionItem::ROTATE;

    solutionItem.fromPose = pose;
    solutionItem.toPose = pose;

    solutionItem.fromPose.theta = AngleMath::normalizeRad(theta0);
    solutionItem.toPose.theta = AngleMath::normalizeRad(thetaf);

    return new Solution<GridSolutionItem>(solutionItem);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<GridSolutionItem>* GridSolutionFactory::fromSearch(BaseMap* map,
    SearchNode<Grid2d>* goalNode)
{
    if (!map)
    {
        return nullptr;
    }

    Solution<GridSolutionItem>* result = new Solution<GridSolutionItem>();

    // If no path was found in the previous search,
    // exit immediately and return an empty solution
    if (!goalNode)
    {
        return result;
    }

    GridSolutionItem solutionItem;
    Pose<> fromPose;
    Pose<> toPose;

    double fromX = 0;
    double fromY = 0;
    double fromTheta = 0;

    double toX = 0;
    double toY = 0;
    double toTheta = 0;

    SearchNode<Grid2d>* toCursor = goalNode;
    SearchNode<Grid2d>* fromCursor = goalNode->parent;

    bool insertNode;

    while (toCursor)
    {
        insertNode = true;

        map->getWorldCoordinates(
            toCursor->action->position.location.x, toCursor->action->position.location.y,
            toX, toY);

        toTheta = AngleMath::deg2Rad<double>(toCursor->action->position.orientation);
        toPose = Pose<>(toX, toY, AngleMath::normalizeRad<double>(toTheta));

        if (fromCursor)
        {
            map->getWorldCoordinates(
                fromCursor->action->position.location.x, fromCursor->action->position.location.y,
                fromX, fromY);

            fromTheta = AngleMath::deg2Rad<double>(fromCursor->action->position.orientation);
            fromPose = Pose<>(fromX, fromY, AngleMath::normalizeRad<double>(fromTheta));
        }

        switch (toCursor->action->actionType)
        {
            case SearchAction<Grid2d>::START:
                insertNode = false;
                break;

            case SearchAction<Grid2d>::GOAL:
                insertNode = false;
                break;

            case SearchAction<Grid2d>::FORWARD:
                solutionItem.actionType = GridSolutionItem::MOVE;
                break;

            case SearchAction<Grid2d>::BACKWARD:
                solutionItem.actionType = GridSolutionItem::MOVE;
                break;

            case SearchAction<Grid2d>::ROTATE_N90:
                solutionItem.actionType = GridSolutionItem::ROTATE;
                fromTheta = AngleMath::normalizeRad<double>(
                    toTheta + AngleMath::deg2Rad<double>(90));
                fromPose = Pose<>(toX, toY, fromTheta);
                break;

            case SearchAction<Grid2d>::ROTATE_P90:
                solutionItem.actionType = GridSolutionItem::ROTATE;
                fromTheta = AngleMath::normalizeRad<double>(
                    toTheta - AngleMath::deg2Rad<double>(90));
                fromPose = Pose<>(toX, toY, fromTheta);
                break;

            case SearchAction<Grid2d>::ROTATE_180:
                solutionItem.actionType = GridSolutionItem::ROTATE;
                fromTheta = AngleMath::normalizeRad<double>(
                    toTheta - AngleMath::deg2Rad<double>(180));
                fromPose = Pose<>(toX, toY, fromTheta);
                break;

            default:
                // It should never see a NONE
                throw;
        }

        solutionItem.fromPose = fromPose;
        solutionItem.toPose = toPose;
        solutionItem.cost = toCursor->action->getTotalCost();

        if (insertNode)
        {
            result->insert(result->begin(), solutionItem);
        }

        toCursor = toCursor->parent;
        if (fromCursor)
        {
            fromCursor = fromCursor->parent;
        }
    }

    return result;
}

} // namespace srs

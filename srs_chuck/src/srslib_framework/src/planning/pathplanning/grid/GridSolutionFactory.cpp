#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<GridSolutionItem>* GridSolutionFactory::fromRotation(Pose<> pose, double theta0, double thetaf)
{
    GridSolutionItem solutionItem;

    solutionItem.actionType = GridSolutionItem::ROTATE;

    solutionItem.fromPose = pose;
    solutionItem.toPose = pose;

    solutionItem.fromPose.theta = AngleMath::normalizeAngleRad(theta0);
    solutionItem.toPose.theta = AngleMath::normalizeAngleRad(thetaf);

    return new Solution<GridSolutionItem>(solutionItem);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Solution<GridSolutionItem>* GridSolutionFactory::fromSearch(SearchNode<Grid2d>* goalNode, Map* map)
{
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

        toTheta = AngleMath::deg2rad<double>(toCursor->action->position.orientation);
        toPose = Pose<>(toX, toY, AngleMath::normalizeAngleRad<double>(toTheta));

        if (fromCursor)
        {
            map->getWorldCoordinates(
                fromCursor->action->position.location.x, fromCursor->action->position.location.y,
                fromX, fromY);

            fromTheta = AngleMath::deg2rad<double>(fromCursor->action->position.orientation);
            fromPose = Pose<>(fromX, fromY, AngleMath::normalizeAngleRad<double>(fromTheta));
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

            case SearchAction<Grid2d>::ROTATE_M90:
                solutionItem.actionType = GridSolutionItem::ROTATE;
                fromTheta = AngleMath::normalizeAngleRad<double>(
                    toTheta + AngleMath::deg2rad<double>(90));
                fromPose = Pose<>(toX, toY, fromTheta);
                break;

            case SearchAction<Grid2d>::ROTATE_P90:
                solutionItem.actionType = GridSolutionItem::ROTATE;
                fromTheta = AngleMath::normalizeAngleRad<double>(
                    toTheta - AngleMath::deg2rad<double>(90));
                fromPose = Pose<>(toX, toY, fromTheta);
                break;

            case SearchAction<Grid2d>::ROTATE_180:
                solutionItem.actionType = GridSolutionItem::ROTATE;
                fromTheta = AngleMath::normalizeAngleRad<double>(
                    toTheta - AngleMath::deg2rad<double>(180));
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

#include <srsnode_navigation/global_planner/AStarPotentials.hpp>

#include <srsnode_navigation/global_planner/QuadraticCalculator.hpp>
#include <srsnode_navigation/global_planner/GradientPath.hpp>
#include <srsnode_navigation/global_planner/GridPath.hpp>
#include <srsnode_navigation/global_planner/AStarExpansion.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
AStarPotentials::AStarPotentials(LogicalMap* logicalMap, costmap_2d::Costmap2D* costMap)
{
    costMap_ = costMap;
    logicalMap_ = logicalMap;

    int sizeX = costMap_->getSizeInCellsX();
    int sizeY = costMap_->getSizeInCellsY();

    // potentialCalculator_ = new PotentialCalculator(sizeX, sizeY);
    potentialCalculator_ = new QuadraticCalculator(sizeX, sizeY);

    stateExpander_ = new AStarExpansion(logicalMap, costMap, potentialCalculator_);
    pathBuilder_ = new GridPath(potentialCalculator_); // GradientPath(pCalculator_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool AStarPotentials::calculatePath(
    double start_x, double start_y,
    double goal_x, double goal_y,
    std::vector<std::pair<float, float>>& path,
    float*& potentials)
{
    int sizeX = costMap_->getSizeInCellsX();
    int sizeY = costMap_->getSizeInCellsY();

    potentialCalculator_->setSize(sizeX, sizeY);
    pathBuilder_->setSize(sizeX, sizeY);

    potentials = new float[sizeX * sizeY];

    double wx = start_x;
    double wy = start_y;
    double start_x_d;
    double start_y_d;
    worldToMap(wx, wy, start_x_d, start_y_d);

    wx = goal_x;
    wy = goal_y;
    double goal_x_d;
    double goal_y_d;
    worldToMap(wx, wy, goal_x_d, goal_y_d);


    unsigned int start_x_i;
    unsigned int start_y_i;
    if (!costMap_->worldToMap(start_x, start_y, start_x_i, start_y_i))
    {
        return false;
    }

    unsigned int goal_x_i;
    unsigned int goal_y_i;
    if (!costMap_->worldToMap(goal_x, goal_y, goal_x_i, goal_y_i))
    {
        return false;
    }

    costMap_->setCost(start_x_i, start_y_i, costmap_2d::FREE_SPACE);
    addMapBorder();

    bool found = stateExpander_->calculatePotentials(start_x_d, start_y_d,
        goal_x_d, goal_y_d,
        sizeX * sizeY * 2,
        potentials);

    stateExpander_->clearEndpoint(potentials, goal_x_i, goal_y_i, 2);
    if (found)
    {
        pathBuilder_->getPath(potentials, start_x_d, start_y_d, goal_x_d, goal_y_d, path);
    }

    return found;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStarPotentials::mapToWorld(double mx, double my, double& wx, double& wy)
{
    wx = costMap_->getOriginX() + (mx + 0.5) * costMap_->getResolution();
    wy = costMap_->getOriginY() + (my + 0.5) * costMap_->getResolution();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool AStarPotentials::worldToMap(double wx, double wy, double& mx, double& my)
{
    double origin_x = costMap_->getOriginX();
    double origin_y = costMap_->getOriginY();
    double resolution = costMap_->getResolution();

    if (wx < origin_x || wy < origin_y)
    {
        return false;
    }

    mx = (wx - origin_x) / resolution - 0.5;
    my = (wy - origin_y) / resolution - 0.5;

    if (mx < costMap_->getSizeInCellsX() && my < costMap_->getSizeInCellsY())
    {
        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStarPotentials::addMapBorder()
{
    unsigned char* costs = costMap_->getCharMap();
    int sizeX = costMap_->getSizeInCellsX();
    int sizeY = costMap_->getSizeInCellsY();

    unsigned char* pc = costMap_->getCharMap();
    for (int i = 0; i < sizeX; i++)
    {
        *pc++ = costmap_2d::LETHAL_OBSTACLE;
    }

    pc = costs + (sizeY - 1) * sizeX;
    for (int i = 0; i < sizeX; i++)
    {
        *pc++ = costmap_2d::LETHAL_OBSTACLE;
    }

    pc = costs;
    for (int i = 0; i < sizeY; i++, pc += sizeX)
    {
        *pc = costmap_2d::LETHAL_OBSTACLE;
    }

    pc = costs + sizeX - 1;
    for (int i = 0; i < sizeY; i++, pc += sizeX)
    {
        *pc = costmap_2d::LETHAL_OBSTACLE;
    }
}

} // namespace srs

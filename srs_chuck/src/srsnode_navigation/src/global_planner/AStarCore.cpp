#include <srsnode_navigation/global_planner/AStarCore.hpp>

#include <srsnode_navigation/global_planner/QuadraticCalculator.hpp>
#include <srsnode_navigation/global_planner/GradientPath.hpp>
#include <srsnode_navigation/global_planner/GridPath.hpp>
#include <srsnode_navigation/global_planner/AStarExpansion.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
AStarCore::AStarCore(LogicalMap* logicalMap, costmap_2d::Costmap2D* costMap)
{
    costMap_ = costMap;
    logicalMap_ = logicalMap;

    int sizeX = costMap_->getSizeInCellsX();
    int sizeY = costMap_->getSizeInCellsY();

    pCalculator_ = new PotentialCalculator(sizeX, sizeY); // new QuadraticCalculator(sizeX, sizeY);

    planner_ = new AStarExpansion(logicalMap, costMap, pCalculator_);
    path_maker_ = new GridPath(pCalculator_); // GradientPath(pCalculator_);

    convert_offset_ = 0.5;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool AStarCore::calculatePath(
    double start_x, double start_y,
    double goal_x, double goal_y,
    std::vector<std::pair<float, float>>& path,
    float*& potentials)
{
    int sizeX = costMap_->getSizeInCellsX();
    int sizeY = costMap_->getSizeInCellsY();

    pCalculator_->setSize(sizeX, sizeY);
    path_maker_->setSize(sizeX, sizeY);

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

    bool found = planner_->calculatePotentials(start_x_d, start_y_d,
        goal_x_d, goal_y_d,
        sizeX * sizeY * 2,
        potentials);

    planner_->clearEndpoint(potentials, goal_x_i, goal_y_i, 2);
    if (found)
    {
        path_maker_->getPath(potentials, start_x_d, start_y_d, goal_x_d, goal_y_d, path);
    }

    return found;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStarCore::mapToWorld(double mx, double my, double& wx, double& wy)
{
    wx = costMap_->getOriginX() + (mx+convert_offset_) * costMap_->getResolution();
    wy = costMap_->getOriginY() + (my+convert_offset_) * costMap_->getResolution();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool AStarCore::worldToMap(double wx, double wy, double& mx, double& my)
{
    double origin_x = costMap_->getOriginX();
    double origin_y = costMap_->getOriginY();
    double resolution = costMap_->getResolution();

    if (wx < origin_x || wy < origin_y)
    {
        return false;
    }

    mx = (wx - origin_x) / resolution - convert_offset_;
    my = (wy - origin_y) / resolution - convert_offset_;

    if (mx < costMap_->getSizeInCellsX() && my < costMap_->getSizeInCellsY())
    {
        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStarCore::addMapBorder()
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

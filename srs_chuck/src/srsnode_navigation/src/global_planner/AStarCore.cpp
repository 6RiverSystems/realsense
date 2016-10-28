#include <srsnode_navigation/global_planner/AStarCore.hpp>

#include <srsnode_navigation/global_planner/QuadraticCalculator.hpp>
#include <srsnode_navigation/global_planner/GradientPath.hpp>
#include <srsnode_navigation/global_planner/AStarExpansion.hpp>
#include <srsnode_navigation/global_planner/OrientationFilter.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
AStarCore::AStarCore(costmap_2d::Costmap2D* costmap)
{
    costmap_ = costmap;

    p_calc_ = new QuadraticCalculator(cx, cy);
    planner_ = new AStarExpansion(p_calc_, cx, cy);
    path_maker_ = new GradientPath(p_calc_);
    orientation_filter_ = new OrientationFilter();

    potential_array_ = nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool AStarCore::calculatePath(
    unsigned int start_x, unsigned int start_y,
    unsigned int end_x, unsigned int end_y,
    std::vector<std::pair<float, float>> path)
{
    int nx = costmap_->getSizeInCellsX();
    int ny = costmap_->getSizeInCellsY();

    p_calc_->setSize(nx, ny);
    planner_->setSize(nx, ny);
    path_maker_->setSize(nx, ny);

    delete potential_array_;
    potential_array_ = new float[nx * ny];

    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);

    bool found_legal = planner_->calculatePotentials(
        costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
        nx * ny * 2, potential_array_);

    planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);

    return found_legal &&
        path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStarCore::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
{
    unsigned char* pc = costarr;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr + (ny - 1) * nx;
    for (int i = 0; i < nx; i++)
        *pc++ = value;
    pc = costarr;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
    pc = costarr + nx - 1;
    for (int i = 0; i < ny; i++, pc += nx)
        *pc = value;
}

} // namespace srs

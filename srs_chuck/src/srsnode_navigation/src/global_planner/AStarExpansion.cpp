#include <costmap_2d/cost_values.h>

#include <srsnode_navigation/global_planner/AStarExpansion.hpp>
#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
AStarExpansion::AStarExpansion(LogicalMap* logicalMap, costmap_2d::Costmap2D* costMap,
    PotentialCalculator* pCalculator) :
        Expander(logicalMap, costMap, pCalculator)
{
    setNeutralCost(1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool AStarExpansion::calculatePotentials(
    double start_x, double start_y,
    double end_x, double end_y,
    int cycles,
    float* potentials)
{
    queue_.clear();

    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potentials, potentials + ns_, POT_HIGH);
    potentials[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;

    while (queue_.size() > 0 && cycle < cycles)
    {
        Index top = queue_[0];

        std::pop_heap(queue_.begin(), queue_.end(), GreaterThan());
        queue_.pop_back();

        int i = top.i;
        if (i == goal_i)
        {
            return true;
        }

        Grid2d::BaseType north = 0;
        Grid2d::BaseType east = 0;
        Grid2d::BaseType south = 0;
        Grid2d::BaseType west = 0;

        int x;
        int y;
        index2Coordinates(i, x, y);

        logicalGrid_->getWeights(Grid2d::Location(x, y), north, east, south, west);

        add(potentials, potentials[i], i + 1, east, end_x, end_y);
        add(potentials, potentials[i], i - 1, west, end_x, end_y);
        add(potentials, potentials[i], i + nx_, north, end_x, end_y);
        add(potentials, potentials[i], i - nx_, south, end_x, end_y);
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStarExpansion::add(float* potentials,
    float prev_potential,
    int next_i, Grid2d::BaseType weight,
    int end_x, int end_y)
{
    float nextPotential = potentials[next_i];
    if (nextPotential < POT_HIGH)
    {
        return;
    }

    if (costGrid_[next_i] >= lethal_cost_)
    {
        return;
    }

    float weightContribution = weight * WEIGHT_RATIO;
    if (weight <= Grid2d::WEIGHT_NO_INFORMATION)
    {
        weightContribution = 0;
    }

    nextPotential = pCalculator_->calculatePotential(potentials,
        costGrid_[next_i] + neutral_cost_,
        next_i, prev_potential) + weightContribution;

    potentials[next_i] = nextPotential;

    int x;
    int y;
    index2Coordinates(next_i, x, y);

    float distance = abs(end_x - x) + abs(end_y - y);
    float gh = nextPotential + distance * neutral_cost_;

    queue_.push_back(Index(next_i, gh));
    std::push_heap(queue_.begin(), queue_.end(), GreaterThan());
}

} // namespace srs

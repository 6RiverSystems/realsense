#include <costmap_2d/cost_values.h>

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/datastructure/Location.hpp>

#include <srsnode_navigation/global_planner/potentials/AStarExpansion.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
AStarExpansion::AStarExpansion(LogicalMap* logicalMap, costmap_2d::Costmap2D* costMap,
    PotentialCalculator* pCalculator) :
        Expander(logicalMap, costMap, pCalculator)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool AStarExpansion::calculatePotentials(
    double start_x, double start_y,
    double end_x, double end_y,
    int cycles,
    float* potentials)
{
    queue_.clear();

    int start_i = coordinates2Index(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potentials, potentials + ns_, PotentialCalculator::MAX_POTENTIAL);
    potentials[start_i] = 0;

    int goal_i = coordinates2Index(end_x, end_y);
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

        logicalGrid_->getWeights(Location(x, y), north, east, south, west);

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
    unsigned int currentCost = costGrid_[next_i];

    if (nextPotential < PotentialCalculator::MAX_POTENTIAL)
    {
        return;
    }

    // If the occupancy map has no information on a specific location,
    // and the exploration for unknown location is disabled, the
    // expansion of the node will not continue
    if (!allowUnknown_ && currentCost == costmap_2d::NO_INFORMATION)
    {
        return;
    }

    if (currentCost >= lethalCost_)
    {
        return;
    }

    unsigned int weightContribution = weight * weightRatio_;
    if (weight == Grid2d::WEIGHT_NO_INFORMATION)
    {
        weightContribution = 0;
    }

    int x;
    int y;
    index2Coordinates(next_i, x, y);

    Grid2d::BaseType logicalCost = logicalGrid_->getPayload(Location(x, y));
    if (logicalCost == Grid2d::PAYLOAD_MAX)
    {
        return;
    }

    // No-information in the logical map is different from no-information
    // in the occupancy map. If the logical map does not have any information
    // for a specific location, there is no additional cost attached to it
    if (logicalCost == Grid2d::PAYLOAD_NO_INFORMATION)
    {
        logicalCost = 0;
    }
    logicalCost *= logicalCostRatio_;

    nextPotential = pCalculator_->calculatePotential(potentials,
        currentCost + neutralCost_,
        next_i, prev_potential) + weightContribution + logicalCost;

    potentials[next_i] = nextPotential;

    float distance = abs(end_x - x) + abs(end_y - y);
    float gh = nextPotential + distance * neutralCost_;

    queue_.push_back(Index(next_i, gh));
    std::push_heap(queue_.begin(), queue_.end(), GreaterThan());
}

} // namespace srs

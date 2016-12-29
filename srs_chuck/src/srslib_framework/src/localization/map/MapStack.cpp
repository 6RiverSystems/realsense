#include <srslib_framework/localization/map/MapStack.hpp>

#include <costmap_2d/cost_values.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapStack::MapStack(LogicalMap* logical, OccupancyMap* occupancy, costmap_2d::Costmap2D* costMap2d) :
    logical_(logical),
    occupancy_(occupancy),
    costMap2d_(costMap2d)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
MapStack::~MapStack()
{
    delete logical_;
    delete occupancy_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
costmap_2d::Costmap2D* MapStack::getCostMap2d() const
{
    return costMap2d_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* MapStack::getLogicalMap() const
{
    return logical_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapStack::getNeighbor(const Grid2d::Position& position, Grid2d::Position& result) const
{
    return logical_->getGrid()->getNeighbor(position, result);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* MapStack::getOccupancyMap() const
{
    return occupancy_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int MapStack::getTotalCost(const Grid2d::Position& position,
    bool allowUnknown,
    float costMapRatio) const
{
    int cost = 0;

    if (logical_)
    {
        cost = logical_->getGrid()->getPayload(position);
        if (cost == Grid2d::PAYLOAD_NO_INFORMATION)
        {
            cost = 0;
        }
    }

    float costMap2d = 0;
    if (costMap2d_)
    {
        costMap2d = costMap2d_->getCost(position.x, position.y) * costMapRatio;
        if (costMap2d == costmap_2d::NO_INFORMATION)
        {
            if (!allowUnknown)
            {
                return Grid2d::PAYLOAD_MAX;
            }
            costMap2d = 0;
        }
    }

    return cost + static_cast<int>(costMap2d);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int MapStack::getWeight(const Grid2d::Position& position) const
{
    int cost = logical_->getGrid()->getWeight(position);

    if (cost == Grid2d::WEIGHT_NO_INFORMATION)
    {
        cost = 0;
    }

    return cost;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapStack::setCostMap2d(costmap_2d::Costmap2D* costMap2d)
{
    costMap2d_ = costMap2d;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

////////////////////////////////////////////////////////////////////////////////////////////////////
// Global operators

} // namespace srs
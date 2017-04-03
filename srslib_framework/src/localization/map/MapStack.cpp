#include <srslib_framework/localization/map/MapStack.hpp>

#include <costmap_2d/cost_values.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MapStack::MapStack(MapStackMetadata metadata,
    LogicalMap* logical, OccupancyMap* occupancy,
    costmap_2d::Costmap2D* costMap2d) :
        metadata_(metadata),
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
MapStackMetadata MapStack::getMetadata() const
{
    return metadata_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool MapStack::getNeighbor(const Position& position, Position& result) const
{
    return logical_->getNeighbor(position, result);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* MapStack::getOccupancyMap() const
{
    return occupancy_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int MapStack::getTotalCost(const Position& position,
    bool allowUnknown,
    float costMapRatio) const
{
    int cost = 0;

    if (logical_)
    {
        cost = logical_->getCost(position);
    }

    float costMap2d = 0;
    if (costMap2d_)
    {
        costMap2d = costMap2d_->getCost(position.location.x, position.location.y) * costMapRatio;
        if (costMap2d == costmap_2d::NO_INFORMATION)
        {
            if (!allowUnknown)
            {
                return SimpleGrid2d::PAYLOAD_MAX;
            }
            costMap2d = 0;
        }
    }

    return cost + static_cast<int>(costMap2d);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
int MapStack::getWeight(const Position& position) const
{
    return logical_->getWeight(position);
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
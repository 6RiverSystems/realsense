/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
using namespace std;

#include <srslib_framework/localization/map/BaseMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMetadata.hpp>

namespace srs {

class OccupancyMap : public BaseMap
{
public:
    static const int8_t COST_INT8_MAX;
    static const unsigned char COST_UCHAR_MAX;

    OccupancyMap(unsigned int widthCells, unsigned int heightCells, double resolution);
    OccupancyMap(OccupancyMetadata metadata);
    ~OccupancyMap()
    {}

    Grid2d::BaseType getCost(unsigned int c, unsigned int r) const
    {
        return getGrid()->getPayload(Grid2d::Location(c, r));
    }

    OccupancyMetadata getMetadata() const
    {
        return occupancyMetadata_;
    }

    Grid2d::BaseType grayLevel2Cost(unsigned char level) const
    {
        Grid2d::BaseType maxCost = numeric_limits<unsigned char>::max();
        Grid2d::BaseType newCost = static_cast<Grid2d::BaseType>(
            occupancyMetadata_.negate ? maxCost - level : level);

        // If the percentage is under the free-cell threshold, the cost is minimal
        float percentage = static_cast<float>(newCost) / static_cast<float>(maxCost);
        newCost = percentage < occupancyMetadata_.thresholdFree ?
            Grid2d::PAYLOAD_MIN :
            newCost;

        // If the percentage is above the occupied-cell threshold, the cost is the maximum
        // allowed cost
        newCost = percentage > occupancyMetadata_.thresholdOccupied ?
            Grid2d::PAYLOAD_MAX :
            newCost;

        return newCost;
    }

    int8_t cost2grayLevel(Grid2d::BaseType intCost) const
    {
        int8_t newCost = static_cast<int8_t>(intCost);
        return occupancyMetadata_.negate ? COST_INT8_MAX - newCost : newCost;
    }

    void maxCost(unsigned int c, unsigned int r, Grid2d::BaseType cost);

    friend ostream& operator<<(ostream& stream, const OccupancyMap& map);
    friend bool operator==(const OccupancyMap& lhs, const OccupancyMap& rhs);

    void setCost(unsigned int c, unsigned int r, Grid2d::BaseType cost);
    void setObstruction(unsigned int c, unsigned int r);

protected:
    OccupancyMetadata occupancyMetadata_;
};

ostream& operator<<(ostream& stream, const OccupancyMap& map);

} // namespace srs

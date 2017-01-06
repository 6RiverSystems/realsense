/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
using namespace std;

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/localization/map/BaseMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMetadata.hpp>

namespace srs {

class OccupancyMap : public BaseMap
{
public:
    static const int8_t COST_INT8_MAX;
    static const unsigned char COST_UCHAR_MAX;

    OccupancyMap(unsigned int widthCells, unsigned int heightCells,
        double resolution, Pose<> origin);
    OccupancyMap(Grid2d* grid, double resolution, Pose<> origin);
    OccupancyMap(OccupancyMetadata metadata);
    ~OccupancyMap()
    {}

    Grid2d::BaseType getCost(unsigned int cCells, unsigned int rCells) const
    {
        return getGrid()->getPayload(Location(cCells, rCells));
    }

    OccupancyMetadata getMetadata() const
    {
        return metadata_;
    }

    Grid2d::BaseType gray2Cost(unsigned char level) const
    {
        float colorAverage = static_cast<float>(level);
        colorAverage = metadata_.negate ? 255.0 - colorAverage : colorAverage;
        float percentage = (255.0 - static_cast<float>(colorAverage)) / 255.0;

        // If the percentage is above the occupied-cell threshold, the cost is
        // the maximum allowed cost
        Grid2d::BaseType newCost = Grid2d::PAYLOAD_NO_INFORMATION;

        if (percentage > metadata_.thresholdOccupied)
        {
            newCost = Grid2d::PAYLOAD_MAX;
        }

        // If the percentage is under the free-cell threshold, the cost is minimal
        if (percentage < metadata_.thresholdFree)
        {
            newCost = Grid2d::PAYLOAD_MIN;
        }

        return newCost;
    }

    Grid2d::BaseType rgb2Cost(unsigned char r, unsigned char g, unsigned char b) const
    {
        float colorAverage = (static_cast<float>(r) + static_cast<float>(g) +
            static_cast<float>(b)) / 3.0;
        colorAverage = metadata_.negate ? 255.0 - colorAverage : colorAverage;
        float percentage = (255.0  - static_cast<float>(colorAverage)) / 255.0;

        Grid2d::BaseType newCost = Grid2d::PAYLOAD_NO_INFORMATION;

        // If the percentage is above the occupied-cell threshold, the cost is
        // the maximum allowed cost
        if (percentage > metadata_.thresholdOccupied)
        {
            newCost = Grid2d::PAYLOAD_MAX;
        }

        // If the percentage is under the free-cell threshold, the cost is minimal
        if (percentage < metadata_.thresholdFree)
        {
            newCost = Grid2d::PAYLOAD_MIN;
        }

        return newCost;
    }

    Grid2d::BaseType rgba2Cost(unsigned char r, unsigned char g, unsigned char b, unsigned char a) const
    {
        float colorAverage = (static_cast<float>(r) + static_cast<float>(g) +
            static_cast<float>(b) + static_cast<float>(a)) / 4.0;
        colorAverage = metadata_.negate ? 255.0 - colorAverage : colorAverage;
        float percentage = (255.0  - static_cast<float>(colorAverage)) / 255.0;

        Grid2d::BaseType newCost = Grid2d::PAYLOAD_NO_INFORMATION;

        // If the percentage is above the occupied-cell threshold, the cost is
        // the maximum allowed cost
        if (percentage > metadata_.thresholdOccupied)
        {
            newCost = Grid2d::PAYLOAD_MAX;
        }

        // If the percentage is under the free-cell threshold, the cost is minimal
        if (percentage < metadata_.thresholdFree)
        {
            newCost = Grid2d::PAYLOAD_MIN;
        }

        return newCost;
    }

    void maxCost(unsigned int cCells, unsigned int rCells, Grid2d::BaseType cost);

    friend ostream& operator<<(ostream& stream, const OccupancyMap& map);
    friend bool operator==(const OccupancyMap& lhs, const OccupancyMap& rhs);

    void setCost(unsigned int cCells, unsigned int rCells, Grid2d::BaseType cost);
    void setObstacle(unsigned int cCells, unsigned int rCells);

protected:
    OccupancyMetadata metadata_;
};

ostream& operator<<(ostream& stream, const OccupancyMap& map);

} // namespace srs

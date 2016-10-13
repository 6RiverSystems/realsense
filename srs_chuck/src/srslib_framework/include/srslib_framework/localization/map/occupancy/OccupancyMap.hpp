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
    OccupancyMap(unsigned int widthCells, unsigned int heightCells, double resolution);
    ~OccupancyMap()
    {}

    int getCost(unsigned int c, unsigned int r) const
    {
        return getGrid()->getCost(Grid2d::Location(c, r));
    }

    OccupancyMetadata getMetadata() const
    {
        return occupancyMetadata_;
    }

    void maxCost(unsigned int c, unsigned int r, int cost);

    friend ostream& operator<<(ostream& stream, const OccupancyMap& map);

    void setCost(unsigned int c, unsigned int r, int cost);

    void setLoadTime(double loadTime)
    {
        occupancyMetadata_.loadTime = loadTime;
    }

    void setNegate(bool negate)
    {
        occupancyMetadata_.negate = negate;
    }

    void setObstruction(unsigned int c, unsigned int r);

    void setOccupancyFilename(string filename)
    {
        occupancyMetadata_.occupancyFilename = filename;
    }

    void setOrigin(Pose<> origin)
    {
        occupancyMetadata_.origin = origin;
    }

    void setThresholds(double free, double occupied)
    {
        occupancyMetadata_.thresholdFree = free;
        occupancyMetadata_.thresholdOccupied = occupied;
    }

protected:
    OccupancyMetadata occupancyMetadata_;
};

ostream& operator<<(ostream& stream, const OccupancyMap& map);

} // namespace srs
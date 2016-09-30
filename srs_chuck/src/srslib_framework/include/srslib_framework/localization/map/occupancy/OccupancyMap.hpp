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
    OccupancyMap();
    OccupancyMap(double widthMeters, double heightMeters, double resolution);
    OccupancyMap(OccupancyMetadata metadata);

    ~OccupancyMap();

    OccupancyMetadata getMetadata() const
    {
        return occupancyMetadata_;
    }

    friend ostream& operator<<(ostream& stream, const OccupancyMap& map);

    void setCost(int c, int r, unsigned int cost);
    void setObstruction(int c, int r);

protected:
    OccupancyMetadata occupancyMetadata_;
};

ostream& operator<<(ostream& stream, const OccupancyMap& map);

} // namespace srs

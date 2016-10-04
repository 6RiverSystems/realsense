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
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

class MapStack
{
public:
    MapStack() :
        logical_(nullptr),
        occupancy_(nullptr)
    {}

    MapStack(LogicalMap* logical, OccupancyMap* occupancy) :
        logical_(logical),
        occupancy_(occupancy)
    {}

    ~MapStack()
    {
        delete logical_;
        delete occupancy_;
    }

    LogicalMap* getLogicalMap() const
    {
        return logical_;
    }

    OccupancyMap* getObstructionMap() const
    {
        // TODO: The obstruction map is in general of type OccupancyMap. However,
        // for now, we simply return the occupancy map instead. There will be a
        // method to merge in new information to generate a new obstruction map
        return occupancy_;
    }

    OccupancyMap* getOccupancyMap() const
    {
        return occupancy_;
    }

private:
    LogicalMap* logical_;
    OccupancyMap* occupancy_;
};

} // namespace srs

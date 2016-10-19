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
#include <srslib_framework/localization/map/logical/LogicalMetadata.hpp>

namespace srs {

class LogicalMap : public BaseMap
{
public:
    LogicalMap(double widthM, double heightM, double resolution);
    ~LogicalMap()
    {}

    int getCost(unsigned int c, unsigned int r) const
    {
        return getGrid()->getPayload(Grid2d::Location(c, r));
    }

    LogicalMetadata getMetadata() const
    {
        return logicalMetadata_;
    }

    int getWeights(unsigned int c, unsigned int r, int orientation) const
    {
        return getGrid()->getWeight(Grid2d::Location(c, r), orientation);
    }

    void maxCost(unsigned int c, unsigned int r, int cost);

    friend ostream& operator<<(ostream& stream, const LogicalMap& map);

    void setCost(unsigned int c, unsigned int r, int cost);
    void setObstruction(unsigned int c, unsigned int r);

    void setLoadTime(double loadTime)
    {
        logicalMetadata_.loadTime = loadTime;
    }

    void setLogicalFilename(string filename)
    {
        logicalMetadata_.logicalFilename = filename;
    }

    void setOrigin(Pose<> origin)
    {
        logicalMetadata_.origin = origin;
    }

    void setWeights(unsigned int c, unsigned int r, int north, int east, int south, int west);

private:
    LogicalMetadata logicalMetadata_;
};

ostream& operator<<(ostream& stream, const LogicalMap& map);

} // namespace srs

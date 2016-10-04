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
    LogicalMap(unsigned int widthCells, unsigned int heightCells, double resolution);
    ~LogicalMap()
    {}

    unsigned int getCost(unsigned int c, unsigned int r) const
    {
        return 0;
    }

    LogicalMetadata getMetadata() const
    {
        return logicalMetadata_;
    }

    friend ostream& operator<<(ostream& stream, const LogicalMap& map);

    void setCost(int c, int r, unsigned int cost);
    void setObstruction(int c, int r);

private:
    LogicalMetadata logicalMetadata_;
};

ostream& operator<<(ostream& stream, const LogicalMap& map);

} // namespace srs

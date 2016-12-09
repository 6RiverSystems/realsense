/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
using namespace std;

#include <srslib_framework/localization/map/mapnote/MapNote.hpp>

namespace srs {

class MaxVelocity : public MapNote
{
public:
    MaxVelocity(float maxVelocity) :
        MapNote(MapNote::MAX_VELOCITY),
        maxVelocity_(maxVelocity)
    {}

    MaxVelocity(string maxVelocity) :
        MapNote(MapNote::MAX_VELOCITY),
        maxVelocity_(stof(maxVelocity))
    {}

    virtual ~MaxVelocity()
    {}

    string getValueString() const
    {
        return to_string(maxVelocity_);
    }

    ostream& toString(ostream& stream) const
    {
        return MapNote::toString(stream) << ", mV: " << maxVelocity_;
    }

    float getMaxVelocity() const
    {
        return maxVelocity_;
    }

private:
    float maxVelocity_;
};

} // namespace srs

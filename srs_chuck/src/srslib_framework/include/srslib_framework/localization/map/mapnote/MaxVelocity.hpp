/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
using namespace std;

#include <srslib_framework/localization/map/mapnote/MapNote2.hpp>

namespace srs {

class MaxVelocity : public MapNote2
{
public:
    static const MaxVelocity MAX_VELOCITY;

    MaxVelocity(float maxVelocity) :
        MapNote2(MapNote2::MAX_VELOCITY, true),
        maxVelocity_(maxVelocity)
    {}

    virtual ~MaxVelocity()
    {}

    friend ostream& operator<<(ostream& stream, const MaxVelocity& note)
    {
        return stream << ", maxV: " << note.maxVelocity_;
    }

    float getMaxVelocity() const
    {
        return maxVelocity_;
    }

private:
    float maxVelocity_;
};

} // namespace srs

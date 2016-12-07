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

class WarningSound : public MapNote2
{
public:
    static const WarningSound WARNING_SOUND;

    WarningSound() :
        MapNote2(MapNote2::WARNING_SOUND, false)
    {}

    virtual ~WarningSound()
    {}
};

} // namespace srs

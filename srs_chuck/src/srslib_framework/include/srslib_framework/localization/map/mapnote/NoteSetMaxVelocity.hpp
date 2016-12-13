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

class NoteSetMaxVelocity : public MapNote
{
public:
    static const string TYPE;

    NoteSetMaxVelocity(float maxVelocity) :
        MapNote(),
        maxVelocity_(maxVelocity)
    {}

    NoteSetMaxVelocity(string maxVelocity) :
        MapNote(),
        maxVelocity_(stof(maxVelocity))
    {}

    virtual ~NoteSetMaxVelocity()
    {}

    string getType() const
    {
        return TYPE;
    }

    string getValue() const
    {
        return to_string(maxVelocity_);
    }

    ostream& toString(ostream& stream) const
    {
        return stream << "type: " << getType() << ", maxVelocity: " << getValue();
    }

    float getMaxVelocity() const
    {
        return maxVelocity_;
    }

private:
    float maxVelocity_;
};

} // namespace srs

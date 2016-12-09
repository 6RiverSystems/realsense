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

class Sound : public MapNote
{
public:
    enum SoundTypeEnum {
        NONE = 0,
        WARNING
    };

    Sound(SoundTypeEnum sound) :
        MapNote(MapNote::SOUND),
        soundType_(sound)
    {}

    Sound(string sound) :
        MapNote(MapNote::SOUND),
        soundType_(static_cast<SoundTypeEnum>(STRING_2_SOUND_TYPE[sound]))
    {}

    virtual ~Sound()
    {}

    string getValueString() const
    {
        return SOUND_TYPE_2_STRING[soundType_];
    }

    ostream& toString(ostream& stream) const
    {
        return MapNote::toString(stream) << ", s: " << soundType_;
    }

    SoundTypeEnum getSoundType() const
    {
        return soundType_;
    }

private:
    static const string SOUND_NONE;
    static const string SOUND_WARNING;

    static unordered_map<int, string> SOUND_TYPE_2_STRING;
    static unordered_map<string, int> STRING_2_SOUND_TYPE;

    SoundTypeEnum soundType_;
};

} // namespace srs

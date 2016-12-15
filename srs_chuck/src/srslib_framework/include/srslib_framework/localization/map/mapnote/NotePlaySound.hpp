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

class NotePlaySound : public MapNote
{
public:
    static const string TYPE;

    enum class SoundType : unsigned char {
        NONE = 0,
        WARNING = 1
    };

    NotePlaySound() :
        MapNote(),
        soundType_(SoundType::NONE)
    {}

    NotePlaySound(SoundType sound) :
        MapNote(),
        soundType_(sound)
    {}

    NotePlaySound(string sound) :
        MapNote(),
        soundType_(static_cast<SoundType>(STRING_2_SOUND_TYPE[sound]))
    {}

    virtual ~NotePlaySound()
    {}

    string getType() const
    {
        return TYPE;
    }

    string getValue() const
    {
        return SOUND_TYPE_2_STRING[static_cast<int>(soundType_)];
    }

    ostream& toString(ostream& stream) const
    {
        return stream << "type: " << getType() << ", soundType: " << getValue();
    }

    SoundType getSoundType() const
    {
        return soundType_;
    }

private:
    static const string SOUND_NONE;
    static const string SOUND_WARNING;

    static unordered_map<int, string> SOUND_TYPE_2_STRING;
    static unordered_map<string, int> STRING_2_SOUND_TYPE;

    SoundType soundType_;
};

} // namespace srs

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <memory>
#include <string>
#include <sstream>
#include <unordered_map>
using namespace std;

namespace srs {

class MapNotes;

class MapNote
{
public:
    enum NoteTypeEnum {
        NONE = 0,
        MAX_VELOCITY,
        SOUND
    };

    using BaseMapNoteType = shared_ptr<MapNote>;

    static BaseMapNoteType instanceOf(const NoteTypeEnum nodeType, string value);
    static BaseMapNoteType instanceOf(const string& field, string value);

    MapNote(NoteTypeEnum noteType = NONE) :
        noteType_(noteType)
    {}

    virtual ~MapNote()
    {}

    NoteTypeEnum getType() const
    {
        return noteType_;
    }

    string getTypeString() const
    {
        return NOTE_TYPE_2_STRING[noteType_];
    }

    virtual string getValueString() const = 0;

    friend ostream& operator<<(ostream& stream, const MapNote& note)
    {
        stream << "{";
        note.toString(stream);
        return stream << "}";
    }

    virtual ostream& toString(ostream& stream) const
    {
        return stream << getTypeString();
    }

private:
    friend class MapNotes;

    static const string NOTE_MAX_VELOCITY;
    static const string NOTE_NONE;
    static const string NOTE_SOUND;

    static unordered_map<int, string> NOTE_TYPE_2_STRING;

    NoteTypeEnum noteType_;
};

} // namespace srs

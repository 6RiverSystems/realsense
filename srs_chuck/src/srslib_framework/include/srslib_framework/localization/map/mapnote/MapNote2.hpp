/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
#include <unordered_map>
using namespace std;

namespace srs {

class MapNotes;

class MapNote2
{
public:
    enum NoteTypeEnum {
        NONE = 0,
        WARNING_SOUND,
        MAX_VELOCITY
    };

    static MapNote2* instanceOf(const MapNote2& note);
    static const MapNote2* instanceOf(const string& field);
    static MapNote2* instanceOf(const string& field, float value);

    bool isMutable() const
    {
        return mutableNote_;
    }

    friend ostream& operator<<(ostream& stream, const MapNote2& note)
    {
        stream << "type: ";
        stream << NOTE_TYPE_2_STRING[note.noteType_];
        return stream << ", mutable: " << note.mutableNote_;
    }

    friend bool operator==(const MapNote2& lhs, const MapNote2& rhs)
    {
        return lhs.noteType_ == rhs.noteType_;
    }

    // MapNote2's equal operation
    bool operator==(const MapNote2 &other) const
    {
        if (this == &other)
        {
            return true;
        }

        return mutableNote_ == other.mutableNote_ &&
            noteType_ == other.noteType_;
    }

    // MapNote2's hash function (needed for maps and sets)
    size_t operator()(const MapNote2& note) const
    {
        return hash<int>()(static_cast<int>(note.noteType_));
    }

//    // MapNote2's comparison operation (needed for maps and sets)
//    bool operator()(const MapNote2& lhs, const MapNote2& rhs) const
//    {
//        return lhs == rhs;
//    }

protected:
    MapNote2(NoteTypeEnum noteType, bool mutableNote) :
        noteType_(noteType),
        mutableNote_(mutableNote)
    {}

    virtual ~MapNote2()
    {}

private:
    friend class MapNotes;

    static const string NOTE_WARNING_SOUND;
    static const string NOTE_MAX_VELOCITY;

    static const unordered_map<string, const MapNote2*> SYMBOL_2_NOTE;
    static unordered_map<int, string> NOTE_TYPE_2_STRING;

    bool mutableNote_;
    NoteTypeEnum noteType_;
};

} // namespace srs

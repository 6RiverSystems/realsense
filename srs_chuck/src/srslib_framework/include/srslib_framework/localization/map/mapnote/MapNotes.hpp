/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
#include <unordered_map>
#include <iomanip>
using namespace std;

#include <srslib_framework/localization/map/mapnote/MapNote.hpp>

namespace srs {

class MapNotes : public unordered_map<int, MapNote::BaseMapNoteType>
{
public:
    using BaseMapNoteType = MapNote::BaseMapNoteType;
    using MapBaseType = unordered_map<int, BaseMapNoteType>;

    MapNotes()
    {}

    virtual ~MapNotes()
    {}

    // TODO: The add method should throw an exception instead of returning a boolean
    bool add(BaseMapNoteType note);
    bool add(const MapNote::NoteTypeEnum nodeType, string value);
    bool add(const string& field, string value);

    template<typename TYPE>
    shared_ptr<TYPE> get(const MapNote::NoteTypeEnum nodeType) const
    {
        auto note = MapBaseType::find(nodeType);
        if (note == MapBaseType::end())
        {
            return nullptr;
        }

        return static_pointer_cast<TYPE>(note->second);
    }

    bool has(const MapNote::NoteTypeEnum nodeType) const;

    friend bool operator==(const MapNotes& lhs, const MapNotes& rhs)
    {
        if (&lhs == &rhs)
        {
            return true;
        }

        if (lhs.size() != rhs.size())
        {
            return false;
        }

        for (auto note : lhs)
        {
            if (!rhs.has(static_cast<MapNote::NoteTypeEnum>(note.first)))
            {
                return false;
            }
        }

        for (auto note : rhs)
        {
            if (!lhs.has(static_cast<MapNote::NoteTypeEnum>(note.first)))
            {
                return false;
            }
        }

        return true;
    }

    friend ostream& operator<<(ostream& stream, const MapNotes& notes)
    {
        int counter = 0;
        for (auto note : notes)
        {
            stream << setw(4) << counter++ << ": " << *(note.second) << endl;
        }
        return stream;
    }
};

} // namespace srs

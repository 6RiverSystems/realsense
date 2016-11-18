/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
using namespace std;

namespace srs {

class MapNote
{
public:
    static const MapNote BEEP;

    static MapNote* instanceOf(const MapNote& note);

    MapNote() :
        beep_(false)
    {}

    MapNote(bool beep) :
        beep_(beep)
    {}

    void join(const MapNote& note)
    {
        beep_ |= note.beep_;
    }

    inline bool beep() const
    {
        return beep_;
    }

    friend ostream& operator<<(ostream& stream, const MapNote& mapNote)
    {
        stream << "{" <<
            (mapNote.beep_ ? "beep" : "");

        return stream << "}";
    }

    friend bool operator==(const MapNote& lhs, const MapNote& rhs)
    {
        return lhs.beep_ == rhs.beep_;
    }

    inline void reset()
    {
        beep_ = false;
    }

private:

    bool beep_;
};

} // namespace srs

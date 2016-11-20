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
    static const MapNote WARNING_BEEP;

    static MapNote* instanceOf(const MapNote& note);

    MapNote() :
        warning_beep_(false)
    {}

    MapNote(bool warning_beep) :
        warning_beep_(warning_beep)
    {}

    void join(const MapNote& note)
    {
        warning_beep_ |= note.warning_beep_;
    }

    inline bool warning_beep() const
    {
        return warning_beep_;
    }

    friend ostream& operator<<(ostream& stream, const MapNote& mapNote)
    {
        stream << "{" <<
            (mapNote.warning_beep_ ? "warning_beep" : "");

        return stream << "}";
    }

    friend bool operator==(const MapNote& lhs, const MapNote& rhs)
    {
        return lhs.warning_beep_ == rhs.warning_beep_;
    }

    inline void reset()
    {
        warning_beep_ = false;
    }

private:

    bool warning_beep_;
};

} // namespace srs

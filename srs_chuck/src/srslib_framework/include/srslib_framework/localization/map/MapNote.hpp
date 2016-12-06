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
    static const MapNote WARNING_SOUND;

    static MapNote* instanceOf(const MapNote& note);

    MapNote() :
        warningSound_(false)
    {}

    MapNote(bool warningSound) :
        warningSound_(warningSound)
    {}

    void join(const MapNote& note)
    {
        warningSound_ |= note.warningSound_;
    }

    inline bool warning_sound() const
    {
        return warningSound_;
    }

    friend ostream& operator<<(ostream& stream, const MapNote& mapNote)
    {
        stream << "{" <<
            (mapNote.warningSound_ ? "warning_sound" : "");

        return stream << "}";
    }

    friend bool operator==(const MapNote& lhs, const MapNote& rhs)
    {
        return lhs.warningSound_ == rhs.warningSound_;
    }

    inline void reset()
    {
        warningSound_ = false;
    }

private:

    bool warningSound_;
};

} // namespace srs

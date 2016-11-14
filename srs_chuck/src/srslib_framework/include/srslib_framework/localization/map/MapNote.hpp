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
    static const MapNote HONK;

    static MapNote* instanceOf(const MapNote& note);

    MapNote() :
        honk_(false)
    {}

    MapNote(bool honk) :
        honk_(honk)
    {}

    void join(const MapNote& note)
    {
        honk_ |= note.honk_;
    }

    inline bool honk() const
    {
        return honk_;
    }

    friend ostream& operator<<(ostream& stream, const MapNote& mapNote)
    {
        stream << "{" <<
            (mapNote.honk_ ? "honk" : "");

        return stream << "}";
    }

    friend bool operator==(const MapNote& lhs, const MapNote& rhs)
    {
        return lhs.honk_ == rhs.honk_;
    }

    inline void reset()
    {
        honk_ = false;
    }

private:

    bool honk_ : 1;
};

} // namespace srs

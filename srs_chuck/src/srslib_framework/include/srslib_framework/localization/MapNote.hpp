/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MAPNOTE_HPP_
#define MAPNOTE_HPP_

#include <string>
#include <sstream>
using namespace std;

namespace srs {

class MapNote
{
public:
    static const MapNote DISABLE_OD;
    static const MapNote ENABLE_OD;
    static const MapNote GO_SLOW;
    static const MapNote NO_ROTATIONS;
    static const MapNote STATIC_OBSTACLE;

    MapNote() :
        od_(false),
        goSlow_(false),
        noRotations_(false),
        staticObstacle_(false)
    {}

    MapNote(bool od,
            bool goSlow,
            bool noRotations,
            bool staticObstacle) :
        od_(od),
        goSlow_(goSlow),
        noRotations_(noRotations),
        staticObstacle_(staticObstacle)
    {}

    void add(const MapNote& note)
    {
        goSlow_ |= note.goSlow_;
        noRotations_ |= note.noRotations_;
        od_ |= note.od_;
        staticObstacle_ |= note.staticObstacle_;
    }

    inline bool goSlow() const
    {
        return goSlow_;
    }

    inline bool od() const
    {
        return od_;
    }

    inline bool noRotations() const
    {
        return noRotations_;
    }

    inline void reset()
    {
        goSlow_ = false;
        noRotations_ = false;
        od_ = false;
        staticObstacle_ = false;
    }

    inline bool staticObstacle() const
    {
        return staticObstacle_;
    }

    friend ostream& operator<<(ostream& stream, const MapNote& mapNote)
    {
        return stream << "{" <<
            (mapNote.goSlow_ ? "GS " : "   ") <<
            (mapNote.noRotations_ ? "NR " : "   ") <<
            (mapNote.od_ ? "OD " : "   ") <<
            (mapNote.staticObstacle_ ? "SO" : "  ") << "}";
    }

private:
    bool goSlow_;
    bool noRotations_;
    bool od_;
    bool staticObstacle_;
};

} // namespace srs

#endif // MAPNOTE_HPP_

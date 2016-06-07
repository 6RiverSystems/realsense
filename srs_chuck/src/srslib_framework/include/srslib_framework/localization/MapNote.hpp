/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MAPNOTE_HPP_
#define MAPNOTE_HPP_

namespace srs {

class MapNote
{
public:
    const static MapNote DISABLE_OD;
    const static MapNote ENABLE_OD;
    const static MapNote GO_SLOW;
    const static MapNote NO_ROTATIONS;
    const static MapNote STATIC_OBSTACLE;

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

private:
    bool goSlow_;
    bool noRotations_;
    bool od_;
    bool staticObstacle_;
};

} // namespace srs

#endif // MAPNOTE_HPP_

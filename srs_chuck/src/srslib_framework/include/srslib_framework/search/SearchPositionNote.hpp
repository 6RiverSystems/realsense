/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SEARCHPOSITIONNOTE_HPP_
#define SEARCHPOSITIONNOTE_HPP_

namespace srs {

class SearchPositionNote
{
public:
    const static SearchPositionNote DISABLE_OD;
    const static SearchPositionNote ENABLE_OD;
    const static SearchPositionNote GO_SLOW;
    const static SearchPositionNote NO_ROTATIONS;
    const static SearchPositionNote STATIC_OBSTACLE;

    SearchPositionNote() :
        od_(false),
        goSlow_(false),
        noRotations_(false),
        staticObstacle_(false)
    {}

    SearchPositionNote(bool od,
            bool goSlow,
            bool noRotations,
            bool staticObstacle) :
        od_(od),
        goSlow_(goSlow),
        noRotations_(noRotations),
        staticObstacle_(staticObstacle)
    {}

    void add(const SearchPositionNote& note)
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

#endif // SEARCHPOSITIONNOTE_HPP_

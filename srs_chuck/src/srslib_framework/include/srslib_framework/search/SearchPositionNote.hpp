/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SEARCHPOSITIONNOTE_HPP_
#define SEARCHPOSITIONNOTE_HPP_

namespace srs {

struct SearchPositionNote
{
    const static SearchPositionNote STATIC_OBSTACLE;
    const static SearchPositionNote GO_SLOW;
    const static SearchPositionNote NO_ROTATIONS;

    SearchPositionNote(
            bool goSlow,
            bool noRotations,
            bool staticObstacle) :
        goSlow(goSlow),
        noRotations(noRotations),
        staticObstacle(staticObstacle)
    {}

    const bool goSlow;
    const bool noRotations;
    const bool staticObstacle;
};

} // namespace srs

#endif // SEARCHPOSITIONNOTE_HPP_

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
    SearchPositionNote(bool noRotations, unsigned int hazardousCost) :
        noRotations(noRotations),
        hazardousCost(hazardousCost)
    {}

    SearchPositionNote(bool noRotations) :
        noRotations(noRotations),
        hazardousCost(0)
    {}

    SearchPositionNote(unsigned int hazardousCost) :
        noRotations(false),
        hazardousCost(hazardousCost)
    {}

    bool noRotations;
    int hazardousCost;
};

} // namespace srs

#endif // SEARCHPOSITIONNOTE_HPP_

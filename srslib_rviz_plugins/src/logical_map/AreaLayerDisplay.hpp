/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

#include "PixelLayerDisplay.hpp"

namespace srs {

class AreaLayerDisplay : public PixelLayerDisplay
{
public:
    AreaLayerDisplay(unsigned int order, unsigned int width, unsigned int height,
        double resolution, Ogre::RGBA color);
    virtual ~AreaLayerDisplay();

    void addArea(string label, Rectangle surface, MapNote::BaseMapNoteType note);

private:
    struct LabeledArea
    {
        LabeledArea(string label, Rectangle surface, MapNote::BaseMapNoteType note) :
            label(label),
            surface(surface),
            note(note)
        {}

        string label;

        MapNote::BaseMapNoteType note;

        Rectangle surface;
    };

    vector<LabeledArea> areas;
};

} // namespace srs

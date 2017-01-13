/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include "PixelLayerDisplay.hpp"

namespace srs {

class AreaLayerDisplay : public PixelLayerDisplay
{
public:
    AreaLayerDisplay(unsigned int order, unsigned int width, unsigned int height,
        double resolution, Ogre::RGBA color);
    virtual ~AreaLayerDisplay();

    void fillArea();

private:
};

} // namespace srs

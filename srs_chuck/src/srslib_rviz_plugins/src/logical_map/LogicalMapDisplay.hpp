/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
#include <vector>
using namespace std;

#ifndef Q_MOC_RUN
#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreSharedPtr.h>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#endif

#include <nav_msgs/MapMetaData.h>
#include <ros/time.h>

#include <srslib_framework/MapStack.h>

#include <rviz/display.h>

#include <srslib_framework/ros/tap/TapMapStack.hpp>
#include "PixelLayerDisplay.hpp"
#include "AreaLayerDisplay.hpp"

namespace Ogre {

class ManualObject;

} // namespace Ogre

namespace rviz {

class FloatProperty;
class IntProperty;
class Property;
class RosTopicProperty;
class VectorProperty;

} // namespace rviz

namespace srs {

class LogicalMapDisplay:
    public rviz::Display,
    public Observer<Subscriber<srslib_framework::MapStack>>
{
Q_OBJECT
public:
    LogicalMapDisplay();
    virtual ~LogicalMapDisplay();

    void notified(Subscriber<srslib_framework::MapStack>* subject);

protected:
    virtual void fixedFrameChanged();

    void initializeLayers();

    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();

    virtual void reset();

Q_SIGNALS:
    void mapStackUpdated();

private Q_SLOTS:
    void renderMapStack();

    void updateAlpha();
    void updateLayerSwitches();
    void updateDrawUnder();

private:
    // Explicitly set the order of the layer
    enum {
        BACKGROUND = 0,
        OBSTACLES = 1,
        WEIGHTS_NORTH = 2,
        WEIGHTS_NORTH_EAST = 3,
        WEIGHTS_EAST = 4,
        WEIGHTS_SOUTH_EAST = 5,
        WEIGHTS_SOUTH = 6,
        WEIGHTS_SOUTH_WEST = 7,
        WEIGHTS_WEST = 8,
        WEIGHTS_NORTH_WEST = 9,
        PLAY_SOUND = 6,
        SET_MAX_VELOCITY = 7
    } EnititesEnum;

    static constexpr Ogre::RGBA RGBA_BLACK = 0x000000FF;
    static constexpr Ogre::RGBA RGBA_BLUE = 0x2874C4FF;
    static constexpr Ogre::RGBA RGBA_GREEN = 0x30845CFF;
    static constexpr Ogre::RGBA RGBA_ORANGE = 0xF0BE48FF;
    static constexpr Ogre::RGBA RGBA_WHITE = 0xFFFFFFFF;

    PixelLayerDisplay* createPixelLayer(unsigned int order, unsigned int width, unsigned int height,
        double resolution, Ogre::RGBA color);
    AreaLayerDisplay* createAreaLayer(unsigned int order, unsigned int width, unsigned int height,
        double resolution, Ogre::RGBA color);

    void translateLogicalMap();

    rviz::FloatProperty* propertyAlpha_;
    rviz::IntProperty* propertyHeight_;
    rviz::Property* propertyLayerBackground_;
    rviz::Property* propertyLayerObstacles_;
    rviz::Property* propertyLayerPlaySound_;
    rviz::Property* propertyLayerSetMaxVelocity_;
    rviz::Property* propertyLayerWeightsNorth_;
    rviz::Property* propertyLayerWeightsNorthEast_;
    rviz::Property* propertyLayerWeightsEast_;
    rviz::Property* propertyLayerWeightsSouthEast_;
    rviz::Property* propertyLayerWeightsSouth_;
    rviz::Property* propertyLayerWeightsSouthWest_;
    rviz::Property* propertyLayerWeightsWest_;
    rviz::Property* propertyLayerWeightsNorthWest_;
    rviz::VectorProperty* propertyOrigin_;
    rviz::FloatProperty* propertyResolution_;
    rviz::IntProperty* propertyWidth_;

    vector<PixelLayerDisplay*> layers_;
    LogicalMap* logicalMap_;

    MapStack* mapStack_;

    TapMapStack tapMapStack_;
};

} // namespace srs

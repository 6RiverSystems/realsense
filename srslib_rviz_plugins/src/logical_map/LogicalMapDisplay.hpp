/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
#include <vector>

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
class StringProperty;
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

    void synthesizeBackgroundLayer();
    void synthesizeObstacleLayer(Location location);
    void synthesizePlaySoundLayer(LogicalMap::LabeledArea area);
    void synthesizeSetMaxVelocityLayer(LogicalMap::LabeledArea area);
    void synthesizeWeightLayers(Location location);

    void updateAlpha();
    void updateLayerSwitches();
    void updateDrawUnder();

private:
    // Explicitly set the order of the layer. The order is the same
    // order with which the layers will be rendered (obstacles are always
    // on top)
    enum {
        BACKGROUND = 0,
        PLAY_SOUND = 1,
        SET_MAX_VELOCITY = 2,
        STAY_ON_PATH = 3,
        WEIGHTS_NORTH = 4,
        WEIGHTS_NORTH_EAST = 5,
        WEIGHTS_EAST = 6,
        WEIGHTS_SOUTH_EAST = 7,
        WEIGHTS_SOUTH = 8,
        WEIGHTS_SOUTH_WEST = 9,
        WEIGHTS_WEST = 10,
        WEIGHTS_NORTH_WEST = 11,
        QUEUE = 12,
        COST_AREA = 13,
        OBSTACLES = 14
    } EnititesEnum;

    static constexpr Ogre::RGBA RGBA_AZALEA = 0xFFBCBEFF;
    static constexpr Ogre::RGBA RGBA_BLACK = 0x000000FF;
    static constexpr Ogre::RGBA RGBA_BRICK_RED = 0xC44448FF;
    static constexpr Ogre::RGBA RGBA_DENIM = 0x2874C4FF;
    static constexpr Ogre::RGBA RGBA_DEEP_KOAMARU = 0x343074FF;
    static constexpr Ogre::RGBA RGBA_SEA_GREEN = 0x30845CFF;
    static constexpr Ogre::RGBA RGBA_CREAM_CAN = 0xF0BE48FF;
    static constexpr Ogre::RGBA RGBA_VIOLET_RED = 0xBC306CFF;
    static constexpr Ogre::RGBA RGBA_WHITE = 0xFFFFFFFF;

    static constexpr int MAX_LAYERS = 15;

    std::shared_ptr<PixelLayerDisplay> createPixelLayer(unsigned int order,
        unsigned int width, unsigned int height,
        double resolution, Ogre::RGBA color);
    std::shared_ptr<AreaLayerDisplay> createAreaLayer(unsigned int order,
        unsigned int width, unsigned int height,
        double resolution, Ogre::RGBA color);

    void translateLogicalMap();

    rviz::FloatProperty* propertyAlpha_;
    rviz::IntProperty* propertyHeight_;
    rviz::Property* propertyLayerBackground_;
    rviz::Property* propertyLayerCostArea_;
    rviz::Property* propertyLayerObstacles_;
    rviz::Property* propertyLayerPlaySound_;
    rviz::Property* propertyLayerQueue_;
    rviz::Property* propertyLayerSetMaxVelocity_;
    rviz::Property* propertyLayerStayOnPath_;
    rviz::Property* propertyLayerWeightsNorth_;
    rviz::Property* propertyLayerWeightsNorthEast_;
    rviz::Property* propertyLayerWeightsEast_;
    rviz::Property* propertyLayerWeightsSouthEast_;
    rviz::Property* propertyLayerWeightsSouth_;
    rviz::Property* propertyLayerWeightsSouthWest_;
    rviz::Property* propertyLayerWeightsWest_;
	rviz::Property* propertyLayerWeightsNorthWest_;
    rviz::StringProperty* propertyMapFilename_;
    rviz::StringProperty* propertyMapName_;
    rviz::StringProperty* propertyMapVersion_;
	rviz::VectorProperty* propertyOrigin_;
    rviz::FloatProperty* propertyResolution_;
    rviz::IntProperty* propertyWidth_;

    std::vector<shared_ptr<PixelLayerDisplay>> layers_;
    LogicalMap* logicalMap_;

    MapStack* mapStack_;

    TapMapStack tapMapStack_;
};

} // namespace srs

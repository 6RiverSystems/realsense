/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
using namespace std;

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>

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

    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();

    virtual void reset();

Q_SIGNALS:
    void mapStackUpdated();

private Q_SLOTS:
    void renderMapStack();

    void updateAlpha();
    void updateDrawUnder();

private:
    enum {
        OBSTACLE = 0,
        EMPTY = 1
    } EnititesEnum;

    class AlphaSetter: public Ogre::Renderable::Visitor
    {
    public:
        AlphaSetter(float alpha) :
            alphaVector_(alpha, alpha, alpha, alpha)
        {}

        void visit(Ogre::Renderable* rend, ushort lodIndex, bool isDebug, Ogre::Any* pAny = 0)
        {
            rend->setCustomParameter(1, alphaVector_);
        }

    private:
        Ogre::Vector4 alphaVector_;
    };

    string generateKey(string prefix)
    {
        stringstream stringStream;
        stringStream << prefix << keyCounter_;

        return stringStream.str();
    }

    void generatePalette();

    void translateLogicalMap();

    static int keyCounter_;

    Ogre::TexturePtr palette_;
    rviz::FloatProperty* propertyAlpha_;
    rviz::Property* propertyDrawUnder_;
    rviz::IntProperty* propertyHeight_;
    rviz::VectorProperty* propertyOrigin_;
    rviz::FloatProperty* propertyResolution_;
    rviz::IntProperty* propertyWidth_;

    LogicalMap* logicalMap_;

    Ogre::ManualObject* manualObject_;
    MapStack* mapStack_;
    Ogre::MaterialPtr material_;

    TapMapStack tapMapStack_;
    Ogre::TexturePtr texture_;
};

} // namespace srs

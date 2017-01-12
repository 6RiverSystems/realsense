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

namespace Ogre {

class ManualObject;

} // namespace Ogre

namespace srs {

class LayerDisplay
{
public:
    LayerDisplay();
    virtual ~LayerDisplay();

private:
//    string generateKey(string prefix)
//    {
//        stringstream stringStream;
//        stringStream << prefix << keyCounter_;
//
//        return stringStream.str();
//    }

    Ogre::TexturePtr palette_;

    Ogre::ManualObject* manualObject_;
    Ogre::MaterialPtr material_;

    Ogre::TexturePtr texture_;
};

} // namespace srs

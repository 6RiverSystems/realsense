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
#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#endif

#include <rviz/display.h>

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/MapStack.h>

namespace Ogre {

class ManualObject;

} // namespace Ogre

namespace srs {

class LayerDisplay
{
public:
    LayerDisplay(unsigned int order, unsigned int width, unsigned int height,
        double resolution, Ogre::RGBA color);
    virtual ~LayerDisplay();

    void connectTo(Ogre::SceneManager* manager, Ogre::SceneNode* mainScene);

    void fillAll();
    void fillLocation(Location location);

    void render();

    void show(bool newState);

    void updateAlpha(float newAlpha);

    Ogre::SceneNode* getScene()
    {
        return scene_;
    }

private:
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

    void generateMaterial();
    void generateManualObject();
    void generatePalette();

    string generateKey(string prefix)
    {
        stringstream stringStream;
        stringStream << prefix << keyCounter_++;

        return stringStream.str();
    }

    Ogre::ColourValue color_;

    unsigned int height_;

    static int keyCounter_;

    Ogre::TexturePtr palette_;
    unsigned char* pixels_;
    unsigned int pixelsPerLayer_;

    Ogre::ManualObject* manualObject_;
    bool modified_;

    Ogre::MaterialPtr objectMaterial_;
    Ogre::TexturePtr objectTexture_;
    unsigned int order_;

    double resolution_;

    Ogre::SceneNode* scene_;

    unsigned int width_;
};

} // namespace srs

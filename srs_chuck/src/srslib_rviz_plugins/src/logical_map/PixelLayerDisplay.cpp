#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#include <ros/ros.h>

#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/custom_parameter_indices.h>
#include <rviz/ogre_helpers/grid.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/validate_floats.h>
#include <rviz/display_context.h>

#include "PixelLayerDisplay.hpp"

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/datastructure/graph/grid2d/WeightedGrid2d.hpp>
#include <srslib_framework/ros/topics/ChuckTransforms.hpp>

namespace srs {

int PixelLayerDisplay::keyCounter_ = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
PixelLayerDisplay::PixelLayerDisplay(unsigned int order, unsigned int width, unsigned int height,
        double resolution, Ogre::RGBA color) :
    manualObject_(nullptr),
    width_(width),
    height_(height),
    resolution_(resolution),
    pixelsPerLayer_(width * height),
    pixels_(nullptr),
    modified_(true),
    order_(order),
    scene_(nullptr)
{
    color_.setAsRGBA(color);

    // Create the pixels of the layer and set them to be completely transparent
    pixels_ = new unsigned char[pixelsPerLayer_];
}

////////////////////////////////////////////////////////////////////////////////////////////////////
PixelLayerDisplay::~PixelLayerDisplay()
{}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::connectTo(Ogre::SceneManager* manager, Ogre::SceneNode* mainScene)
{
    // Create the movable scene and attach a manual object
    scene_ = manager->createSceneNode();
    manualObject_ = manager->createManualObject(generateKey("LogicalMapLayerObject"));

    scene_->attachObject(manualObject_);
    scene_->setPosition(0.0f, 0.0f, 0.0001f * order_);

    // Attach the scene to the main scene, so that manipulating
    // the main scene, all the other children scenes are manipulated
    // in cascade
    mainScene->addChild(scene_);

    generatePalette();
    generateMaterial();
    generateManualObject();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::fillAll()
{
    memset(pixels_, VALUE_MAX, pixelsPerLayer_);
    modified_ = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::fillLocation(Location location, unsigned char value)
{
    fillLocation(location.x, location.y, value);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::fillArea(unsigned int ciCells, unsigned int riCells,
    unsigned int cfCells, unsigned int rfCells, unsigned char value)
{
    unsigned int r0 = riCells;
    unsigned int c0 = ciCells;
    unsigned int widthCells = cfCells - ciCells;
    unsigned int heightCells = rfCells - riCells;

    for (unsigned int r = r0; r < r0 + heightCells; ++r)
    {
        for (unsigned int c = c0; c < c0 + widthCells; ++c)
        {
            fillLocation(c, r, value);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::fillLocation(unsigned int x, unsigned int y, unsigned char value)
{
    float ratio = static_cast<float>(value) / VALUE_MAX;
    pixels_[x + y * width_] = static_cast<unsigned char>(
        VALUE_MIN + (VALUE_MAX - VALUE_MIN) * ratio);
    modified_ = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::render()
{
    if (manualObject_ && modified_)
    {
        Ogre::DataStreamPtr pixelStream;
        pixelStream.bind(new Ogre::MemoryDataStream(pixels_, pixelsPerLayer_));

        if (!objectTexture_.isNull())
        {
            Ogre::TextureManager::getSingleton().remove(objectTexture_->getName());
            objectTexture_.setNull();
        }

        // Generate the texture for the stream of pixels
        try
        {
            objectTexture_ = Ogre::TextureManager::getSingleton().loadRawData(
                generateKey("LogicalMapTexture"),
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                pixelStream, width_, height_, Ogre::PF_L8, Ogre::TEX_TYPE_2D, 0);
        }
        catch (Ogre::RenderingAPIException&)
        {
            Ogre::Image image;
            pixelStream->seek(0);

            float scaledWidth = width_;
            float scaledHeight = height_;

            if (width_ > height_)
            {
                float aspect = scaledHeight / scaledWidth;
                scaledWidth = 2048;
                scaledHeight = scaledWidth * aspect;
            }
            else
            {
                float aspect = scaledWidth / scaledHeight;
                scaledHeight = 2048;
                scaledWidth = scaledHeight * aspect;
            }

            std::stringstream ss;
            ss << "Map is larger than your graphics card supports. Downsampled from [" <<
                width_ << "x" << height_ << "] to [" << scaledWidth << "x" << scaledHeight << "]";

            image.loadRawData(pixelStream, width_, height_, Ogre::PF_L8);
            image.resize(scaledWidth, scaledHeight, Ogre::Image::FILTER_NEAREST);

            objectTexture_ = Ogre::TextureManager::getSingleton().loadImage(
                generateKey("Downsampled"),
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
        }

        // Apply the texture to the material
        Ogre::Pass* pass = objectMaterial_->getTechnique(0)->getPass(0);
        Ogre::TextureUnitState* textureUnit = nullptr;
        if (pass->getNumTextureUnitStates() > 0)
        {
            textureUnit = pass->getTextureUnitState(0);
        }
        else
        {
            textureUnit = pass->createTextureUnitState();
        }
        textureUnit->setTextureName(objectTexture_->getName());
        textureUnit->setTextureFiltering(Ogre::TFO_NONE);

        Ogre::TextureUnitState* paletteUnit = nullptr;
        if (pass->getNumTextureUnitStates() > 1)
        {
            paletteUnit = pass->getTextureUnitState(1);
        }
        else
        {
            paletteUnit = pass->createTextureUnitState();
        }

        paletteUnit->setTextureName(palette_->getName());
        paletteUnit->setTextureFiltering(Ogre::TFO_NONE);

        modified_ = false;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::show(bool newState)
{
    manualObject_->setVisible(newState);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::updateAlpha(float newAlpha)
{
    objectMaterial_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    objectMaterial_->setDepthWriteEnabled(false);

    AlphaSetter alphaSetter(newAlpha);
    if (manualObject_)
    {
        manualObject_->visitRenderables(&alphaSetter);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::generateManualObject()
{
    manualObject_->begin(objectMaterial_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
    {
        // First triangle
        {
          // Bottom left
          manualObject_->position(0.0f, 0.0f, 0.0f);
          manualObject_->textureCoord(0.0f, 0.0f);
          manualObject_->normal(0.0f, 0.0f, 1.0f);

          // Top right
          manualObject_->position(1.0f, 1.0f, 0.0f);
          manualObject_->textureCoord(1.0f, 1.0f);
          manualObject_->normal(0.0f, 0.0f, 1.0f);

          // Top left
          manualObject_->position(0.0f, 1.0f, 0.0f);
          manualObject_->textureCoord(0.0f, 1.0f);
          manualObject_->normal(0.0f, 0.0f, 1.0f);
        }

        // Second triangle
        {
          // Bottom left
          manualObject_->position(0.0f, 0.0f, 0.0f);
          manualObject_->textureCoord(0.0f, 0.0f);
          manualObject_->normal(0.0f, 0.0f, 1.0f);

          // Bottom right
          manualObject_->position(1.0f, 0.0f, 0.0f);
          manualObject_->textureCoord(1.0f, 0.0f);
          manualObject_->normal(0.0f, 0.0f, 1.0f);

          // Top right
          manualObject_->position(1.0f, 1.0f, 0.0f);
          manualObject_->textureCoord(1.0f, 1.0f);
          manualObject_->normal(0.0f, 0.0f, 1.0f);
        }
    }

    manualObject_->end();
    manualObject_->setVisible(true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::generateMaterial()
{
    // Set up map material
    objectMaterial_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
    objectMaterial_ = objectMaterial_->clone(generateKey("LogicalMapMaterial"));
    objectMaterial_->setReceiveShadows(false);
    objectMaterial_->getTechnique(0)->setLightingEnabled(false);
    objectMaterial_->setDepthBias(-16.0f, 0.0f);
    objectMaterial_->setCullingMode(Ogre::CULL_NONE);
    objectMaterial_->setDepthWriteEnabled(false);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void PixelLayerDisplay::generatePalette()
{
    const int PALETTE_SIZE = 256 * 4;

    unsigned char* paletteColors = new unsigned char[PALETTE_SIZE];
    unsigned char* p = paletteColors;

    // Background color (transparent layer)
    *p++ = 0; // red
    *p++ = 0; // green
    *p++ = 0; // blue
    *p++ = 0; // alpha

    for (int i = 0; i < 256; i++)
    {
        *p++ = static_cast<unsigned char>(color_[0] * (static_cast<float>(i) / 255.0) * 255); // red
        *p++ = static_cast<unsigned char>(color_[1] * (static_cast<float>(i) / 255.0) * 255); // green
        *p++ = static_cast<unsigned char>(color_[2] * (static_cast<float>(i) / 255.0) * 255); // blue
        *p++ = 255; // alpha
    }

    Ogre::DataStreamPtr paletteStream;
    paletteStream.bind(new Ogre::MemoryDataStream(paletteColors, PALETTE_SIZE));

    palette_ = Ogre::TextureManager::getSingleton().loadRawData(
        generateKey("LogicalMapPaletteTexture"),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        paletteStream, 256, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0);
}

} // namespace srs

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreTechnique.h>
#include <OgreSharedPtr.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>

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

#include "LogicalMapDisplay.hpp"

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/datastructure/graph/grid2d/WeightedGrid2d.hpp>
#include <srslib_framework/ros/topics/ChuckTransforms.hpp>

namespace srs {

int LogicalMapDisplay::keyCounter_ = 0;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMapDisplay::LogicalMapDisplay() :
    Display(),
    logicalMap_(nullptr),
    mapStack_(nullptr),
    manualObject_(nullptr)
{
    propertyAlpha_ = new rviz::FloatProperty("Alpha", 1.0,
        "0 is fully transparent, 1.0 is fully opaque.",
        this, SLOT(updateAlpha()));
    propertyAlpha_->setMin(0);
    propertyAlpha_->setMax(1);

    propertyDrawUnder_ = new rviz::Property("Draw Behind", false,
        "Rendering option, controls whether or not the map is always"
        " drawn behind everything else.",
        this, SLOT(updateDrawUnder()));

    propertyResolution_ = new rviz::FloatProperty("Resolution", 0,
        "Resolution of the map [meter].", this);
    propertyResolution_->setReadOnly(true);

    propertyWidth_ = new rviz::IntProperty("Width", 0,
        "Width of the map [meter]", this);
    propertyWidth_->setReadOnly(true);

    propertyHeight_ = new rviz::IntProperty( "Height", 0,
        "Height of the map [meter]", this);
    propertyHeight_->setReadOnly(true);

    propertyOrigin_ = new rviz::VectorProperty( "Position", Ogre::Vector3::ZERO,
        "Position of the bottom left corner of the map [meter]", this);
    propertyOrigin_->setReadOnly(true);

    connect(this, SIGNAL(mapStackUpdated()), this, SLOT(renderMapStack()));
    tapMapStack_.attach(this);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMapDisplay::~LogicalMapDisplay()
{}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::notified(Subscriber<srslib_framework::MapStack>* subject)
{
    TapMapStack* tapMapStack = static_cast<TapMapStack*>(subject);

    // When the Map Stack is published, make sure to get a fresh copy
    delete mapStack_;
    mapStack_ = tapMapStack_.pop();

    // Update the read-only properties of the display
    logicalMap_ = mapStack_->getLogicalMap();
    propertyResolution_->setValue(logicalMap_->getResolution());
    propertyWidth_->setValue(logicalMap_->getWidthM());
    propertyHeight_->setValue(logicalMap_->getHeightM());
    propertyOrigin_->setVector(Ogre::Vector3(
        logicalMap_->getOrigin().x,
        logicalMap_->getOrigin().y,
        0));

    setStatus(rviz::StatusProperty::Ok, "Message", "Logical Map received");

    Q_EMIT mapStackUpdated();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Protected methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::fixedFrameChanged()
{
    translateLogicalMap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::onInitialize()
{
    // Generate the palette
    generatePalette();

    // Set up map material
    material_ = Ogre::MaterialManager::getSingleton().getByName("rviz/Indexed8BitImage");
    material_ = material_->clone(generateKey("LogicalMapMaterial"));
    material_->setReceiveShadows(false);
    material_->getTechnique(0)->setLightingEnabled(false);
    material_->setDepthBias(-16.0f, 0.0f);
    material_->setCullingMode(Ogre::CULL_NONE);
    material_->setDepthWriteEnabled(false);

    manualObject_ = scene_manager_->createManualObject(generateKey("LogicalMapObject"));
    scene_node_->attachObject(manualObject_);

    manualObject_->begin(material_->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);
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
    manualObject_->setVisible(false);

    updateDrawUnder();
    updateAlpha();
    translateLogicalMap();

    setEnabled(true);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::onEnable()
{
    if (manualObject_)
    {
        manualObject_->setVisible(true);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::onDisable()
{
    if (manualObject_)
    {
        manualObject_->setVisible(false);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::reset()
{
    Display::reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::generatePalette()
{
    const int PALETTE_SIZE = 256 * 4;

    unsigned char* paletteColors = new unsigned char[PALETTE_SIZE];
    unsigned char* p = paletteColors;

    // Obstacle
    *p++ = 0; // red
    *p++ = 0; // green
    *p++ = 0; // blue
    *p++ = 255; // alpha

    // Empty
    *p++ = 255; // red
    *p++ = 255; // green
    *p++ = 255; // blue
    *p++ = 255; // alpha

//    // Blue to red spectrum for most normal cost values
//    for (int i = 1; i <= 98; i++)
//    {
//        unsigned char v = (255 * i) / 100;
//        *p++ = v; // red
//        *p++ = 0; // green
//        *p++ = 255 - v; // blue
//        *p++ = 255; // alpha
//    }
//
//    // Inscribed obstacle values (99) in cyan
//    *p++ = 0; // red
//    *p++ = 255; // green
//    *p++ = 255; // blue
//    *p++ = 255; // alpha
//
//    // Lethal obstacle values (100) in purple
//    *p++ = 255; // red
//    *p++ = 0; // green
//    *p++ = 255; // blue
//    *p++ = 255; // alpha
//
//    // Illegal positive values in green
//    for (int i = 101; i <= 127; i++)
//    {
//        *p++ = 0; // red
//        *p++ = 255; // green
//        *p++ = 0; // blue
//        *p++ = 255; // alpha
//    }
//
//    // Illegal negative (char) values in shades of red/yellow
//    for (int i = 128; i <= 254; i++)
//    {
//        *p++ = 255; // red
//        *p++ = (255 * (i - 128)) / (254 - 128); // green
//        *p++ = 0; // blue
//        *p++ = 255; // alpha
//    }
//
//    // Legal -1 value is tasteful blueish greenish grayish color
//    *p++ = 0x70; // red
//    *p++ = 0x89; // green
//    *p++ = 0x86; // blue
//    *p++ = 255; // alpha

    Ogre::DataStreamPtr paletteStream;
    paletteStream.bind(new Ogre::MemoryDataStream(paletteColors, PALETTE_SIZE));

    palette_ = Ogre::TextureManager::getSingleton().loadRawData("LogicalMapPaletteTexture",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        paletteStream, 256, 1, Ogre::PF_BYTE_RGBA, Ogre::TEX_TYPE_1D, 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::translateLogicalMap()
{
    if (!logicalMap_)
    {
        return;
    }

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;

    geometry_msgs::Pose origin;
    origin.position.x = logicalMap_->getOrigin().x;
    origin.position.y = logicalMap_->getOrigin().x;
    origin.position.z = 0;
    origin.orientation.x = 0.0;
    origin.orientation.y = 0.0;
    origin.orientation.z = 0.0;
    origin.orientation.w = 1.0;

    if (!context_->getFrameManager()->transform(ChuckTransforms::MAP,
        ros::Time(), origin, position, orientation))
    {
        setStatus(rviz::StatusProperty::Error, "Transform",
            "No transform from [" + QString::fromStdString(ChuckTransforms::MAP) +
            "] to [" + fixed_frame_ + "]" );
    }
    else
    {
        setStatus(rviz::StatusProperty::Ok, "Transform", "Transform OK");
    }

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::renderMapStack()
{
    if (!logicalMap_ || !isEnabled())
    {
        return;
    }

    ROS_WARN("renderMapStack");

    unsigned int width = logicalMap_->getWidthCells();
    unsigned int height = logicalMap_->getHeightCells();
    double resolution = logicalMap_->getResolution();

    unsigned int numberOfPixels = width * height;
    unsigned char* pixels = new unsigned char[numberOfPixels];

    // Set the map to be completely empty
    memset(pixels, EMPTY, numberOfPixels);

    // Scan through the weighted grid of the map and
    // choose the correct color based on the information in the cell
    WeightedGrid2d* grid = logicalMap_->getGrid();
    for (Location location : *grid)
    {
        WeightedGrid2d::BaseType cost = grid->getPayload(location);
        if (cost == WeightedGrid2d::PAYLOAD_MAX)
        {
            int index = location.x + location.y * width;
            pixels[index] = OBSTACLE;
        }
    }

    Ogre::DataStreamPtr pixelStream;
    pixelStream.bind(new Ogre::MemoryDataStream(pixels, numberOfPixels));

    if (!texture_.isNull())
    {
        Ogre::TextureManager::getSingleton().remove(texture_->getName());
        texture_.setNull();
    }

    // Generate the texture for the stream of pixels
    try
    {
        texture_ = Ogre::TextureManager::getSingleton().loadRawData(generateKey("LogicalMapTexture"),
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
            pixelStream, width, height, Ogre::PF_L8, Ogre::TEX_TYPE_2D, 0);
    }
    catch (Ogre::RenderingAPIException&)
    {
        Ogre::Image image;
        pixelStream->seek(0);

        float scaledWidth = width;
        float scaledHeight = height;

        if (width > height)
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
            width << "x" << height << "] to [" << scaledWidth << "x" << scaledHeight << "]";
        setStatus(rviz::StatusProperty::Warn, "Map", QString::fromStdString(ss.str()));

        image.loadRawData(pixelStream, width, height, Ogre::PF_L8);
        image.resize(scaledWidth, scaledHeight, Ogre::Image::FILTER_NEAREST);

        texture_ = Ogre::TextureManager::getSingleton().loadImage(generateKey("Downsampled"),
            Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
    }

    // There is no need for the pixels anymore
    delete[] pixels;

    // Apply the texture to the material
    Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
    Ogre::TextureUnitState* textureUnit = nullptr;
    if (pass->getNumTextureUnitStates() > 0)
    {
        textureUnit = pass->getTextureUnitState(0);
    }
    else
    {
        textureUnit = pass->createTextureUnitState();
    }
    textureUnit->setTextureName(texture_->getName());
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

    updateAlpha();


    // Make sure that the map is displayed in the correct position
    translateLogicalMap();

    // Ask the queue to render the object
    manualObject_->setVisible(true);
    scene_node_->setScale(resolution * width, resolution * height, 1.0);
    context_->queueRender();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::updateAlpha()
{
    float alpha = propertyAlpha_->getFloat();

    material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    material_->setDepthWriteEnabled(false);

    AlphaSetter alphaSetter(alpha);
    if (manualObject_)
    {
        manualObject_->visitRenderables(&alphaSetter);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::updateDrawUnder()
{
    bool drawUnder = propertyDrawUnder_->getValue().toBool();

    if (propertyAlpha_->getFloat() >= 0.9998)
    {
        material_->setDepthWriteEnabled(!drawUnder);
    }

    if (manualObject_)
    {
        if (drawUnder)
        {
            manualObject_->setRenderQueueGroup(Ogre::RENDER_QUEUE_4);
        }
        else
        {
            manualObject_->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
        }
    }
}

} // namespace srs

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(srs::LogicalMapDisplay, rviz::Display)

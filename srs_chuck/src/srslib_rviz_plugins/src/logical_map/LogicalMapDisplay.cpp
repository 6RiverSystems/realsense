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

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMapDisplay::LogicalMapDisplay() :
    Display(),
    logicalMap_(nullptr),
    mapStack_(nullptr)
{
    propertyAlpha_ = new rviz::FloatProperty("Alpha", 1.0,
        "0 is fully transparent, 1.0 is fully opaque.",
        this, SLOT(updateAlpha()));
    propertyAlpha_->setMin(0);
    propertyAlpha_->setMax(1);

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

    propertyLayerBackground_ = new rviz::Property("Draw background layer", true,
        "Rendering option, enables/disables the background layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerObstacles_ = new rviz::Property("Draw obstacles layer", true,
        "Rendering option, enables/disables the obstacles layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsNorth_ = new rviz::Property("Draw north weights layer", true,
        "Rendering option, enables/disables the north weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsEast_ = new rviz::Property("Draw east weights layer", true,
        "Rendering option, enables/disables the east weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsSouth_ = new rviz::Property("Draw south weights layer", true,
        "Rendering option, enables/disables the south weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsWest_ = new rviz::Property("Draw west weights layer", true,
        "Rendering option, enables/disables the west weights layer.",
        this, SLOT(updateLayerSwitches()));

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

    // Now that we know about the logical map,
    // the different layers can be constructed and initialized
    initializeLayers();

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
void LogicalMapDisplay::initializeLayers()
{
    unsigned int width = logicalMap_->getWidthCells();
    unsigned int height = logicalMap_->getHeightCells();
    double resolution = logicalMap_->getResolution();

    PixelLayerDisplay* background = createPixelLayer(BACKGROUND,
        width, height, resolution, RGBA_WHITE);
    PixelLayerDisplay* obstacles = createPixelLayer(OBSTACLES,
        width, height, resolution, RGBA_BLACK);

    PixelLayerDisplay* weightsNorth = createPixelLayer(WEIGHTS_NORTH,
        width, height, resolution, RGBA_ORANGE);
    PixelLayerDisplay* weightsEast = createPixelLayer(WEIGHTS_EAST,
        width, height, resolution, RGBA_ORANGE);
    PixelLayerDisplay* weightsSouth = createPixelLayer(WEIGHTS_SOUTH,
        width, height, resolution, RGBA_ORANGE);
    PixelLayerDisplay* weightsWest = createPixelLayer(WEIGHTS_WEST,
        width, height, resolution, RGBA_ORANGE);

    AreaLayerDisplay* warningSound = createAreaLayer(WARNING_SOUND,
        width, height, resolution, RGBA_GREEN);

    // Background layer
    background->fillAll();

    // Go through all defined cells in the grid and
    // populate the layers
    WeightedGrid2d* grid = logicalMap_->getGrid();
    for (Location location : *grid)
    {
        // Store the obstacle pixels
        WeightedGrid2d::BaseType cost = grid->getPayload(location);
        if (cost == WeightedGrid2d::PAYLOAD_MAX)
        {
            obstacles->fillLocation(location);
        }

        // Store the weights
        WeightedGrid2d::BaseType north;
        WeightedGrid2d::BaseType east;
        WeightedGrid2d::BaseType south;
        WeightedGrid2d::BaseType west;

        grid->getWeights(location, north, east, south, west);
        if (north > 0)
        {
            weightsNorth->fillLocation(location, north);
        }
        if (east > 0)
        {
            weightsEast->fillLocation(location, east);
        }
        if (south > 0)
        {
            weightsSouth->fillLocation(location, south);
        }
        if (west > 0)
        {
            weightsWest->fillLocation(location, west);
        }
    }

    for (auto area : logicalMap_->getAreas())
    {
    }

    // Add all the layers to the stack for rendering
    layers_.push_back(background);
    layers_.push_back(obstacles);
    layers_.push_back(weightsNorth);
    layers_.push_back(weightsEast);
    layers_.push_back(weightsSouth);
    layers_.push_back(weightsWest);
    layers_.push_back(warningSound);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::onInitialize()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::onEnable()
{
    for (auto layer : layers_)
    {
        layer->show(true);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::onDisable()
{
    for (auto layer : layers_)
    {
        layer->show(false);
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
PixelLayerDisplay* LogicalMapDisplay::createPixelLayer(unsigned int order,
    unsigned int width, unsigned int height,
    double resolution, Ogre::RGBA color)
{
    PixelLayerDisplay* layer = new PixelLayerDisplay(order, width, height, resolution, color);
    layer->connectTo(scene_manager_, scene_node_);

    return layer;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
AreaLayerDisplay* LogicalMapDisplay::createAreaLayer(unsigned int order,
    unsigned int width, unsigned int height,
    double resolution, Ogre::RGBA color)
{
    AreaLayerDisplay* layer = new AreaLayerDisplay(order, width, height, resolution, color);
    layer->connectTo(scene_manager_, scene_node_);

    return layer;
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

    for (auto layer : layers_)
    {
        layer->render();
    }

    // Make sure that the alpha of the layers matches the specified one
    updateAlpha();

    // Make sure that the map is displayed in the correct position
    translateLogicalMap();

    // Ask the queue to render the object
    unsigned int width = logicalMap_->getWidthCells();
    unsigned int height = logicalMap_->getHeightCells();
    double resolution = logicalMap_->getResolution();

    scene_node_->setScale(resolution * width, resolution * height, 1.0);
    context_->queueRender();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::updateAlpha()
{
    float alpha = propertyAlpha_->getFloat();

    for (auto layer : layers_)
    {
        layer->updateAlpha(alpha);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::updateLayerSwitches()
{
    layers_[BACKGROUND]->show(propertyLayerBackground_->getValue().toBool());
    layers_[OBSTACLES]->show(propertyLayerObstacles_->getValue().toBool());
    layers_[WEIGHTS_NORTH]->show(propertyLayerWeightsNorth_->getValue().toBool());
}

} // namespace srs

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(srs::LogicalMapDisplay, rviz::Display)

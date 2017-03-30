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
#include <rviz/properties/string_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/validate_floats.h>
#include <rviz/display_context.h>

#include "LogicalMapDisplay.hpp"

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/datastructure/graph/grid2d/WeightedGrid2d.hpp>
#include <srslib_framework/chuck/ChuckTransforms.hpp>
#include <srslib_framework/localization/map/mapnote/NotePlaySound.hpp>
#include <srslib_framework/localization/map/mapnote/NoteQueue.hpp>
#include <srslib_framework/localization/map/mapnote/NoteSetMaxVelocity.hpp>
#include <srslib_framework/localization/map/mapnote/NoteStayOnPath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMapDisplay::LogicalMapDisplay() :
    Display(),
    logicalMap_(nullptr),
    mapStack_(nullptr)
{
    layers_ = std::vector<shared_ptr<PixelLayerDisplay>>(MAX_LAYERS, nullptr);

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

    propertyLayerBackground_ = new rviz::Property("Background layer", true,
        "Rendering option, enables/disables the background layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerCostArea_ = new rviz::Property("Cost area layer", true,
        "Rendering option, enables/disables the cost area layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerObstacles_ = new rviz::Property("Obstacles layer", true,
        "Rendering option, enables/disables the obstacles layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerPlaySound_ = new rviz::Property("Play-sound layer", true,
        "Rendering option, enables/disables the play-sound layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerQueue_ = new rviz::Property("Queue layer", true,
        "Rendering option, enables/disables the queue layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerSetMaxVelocity_ = new rviz::Property("Set-max-velocity layer", true,
        "Rendering option, enables/disables the set-max-velocity layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerStayOnPath_ = new rviz::Property("Stay-on-path layer", true,
        "Rendering option, enables/disables the stay-on-path layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsNorth_ = new rviz::Property("North weights layer", true,
        "Rendering option, enables/disables the north weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsNorthEast_ = new rviz::Property("North-East weights layer", true,
        "Rendering option, enables/disables the north-east weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsEast_ = new rviz::Property("East weights layer", true,
        "Rendering option, enables/disables the east weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsSouthEast_ = new rviz::Property("South-East weights layer", true,
        "Rendering option, enables/disables the south-east weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsSouth_ = new rviz::Property("South weights layer", true,
        "Rendering option, enables/disables the south weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsSouthWest_ = new rviz::Property("South-West weights layer", true,
        "Rendering option, enables/disables the south-west weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsWest_ = new rviz::Property("West weights layer", true,
        "Rendering option, enables/disables the west weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyLayerWeightsNorthWest_ = new rviz::Property("North-West weights layer", true,
        "Rendering option, enables/disables the north-west weights layer.",
        this, SLOT(updateLayerSwitches()));

    propertyMapFilename_ = new rviz::StringProperty( "Map file name", "",
        "File name of the loaded map", this);
    propertyMapFilename_->setReadOnly(true);

    propertyMapName_ = new rviz::StringProperty( "Map name", "",
        "Name assigned to the map", this);
    propertyMapName_->setReadOnly(true);

    propertyMapVersion_ = new rviz::StringProperty( "Map version", "",
        "Version assigned to the map", this);
    propertyMapVersion_->setReadOnly(true);

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

    propertyMapFilename_->setStdString(mapStack_->getMetadata().mapStackFilename);
    propertyMapName_->setStdString(mapStack_->getMetadata().mapName);
    propertyMapVersion_->setStdString(mapStack_->getMetadata().mapVersion);

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

    // Background layer
    layers_[BACKGROUND] = createPixelLayer(BACKGROUND, width, height, resolution, RGBA_WHITE);
    layers_[BACKGROUND]->fillAll();

    layers_[OBSTACLES] = createPixelLayer(OBSTACLES,
        width, height, resolution, RGBA_BLACK);

    layers_[COST_AREA] = createPixelLayer(COST_AREA,
        width, height, resolution, RGBA_AZALEA);

    layers_[WEIGHTS_NORTH] = createPixelLayer(WEIGHTS_NORTH,
        width, height, resolution, RGBA_CREAM_CAN);
    layers_[WEIGHTS_NORTH_EAST] = createPixelLayer(WEIGHTS_NORTH_EAST,
        width, height, resolution, RGBA_CREAM_CAN);
    layers_[WEIGHTS_EAST] = createPixelLayer(WEIGHTS_EAST,
        width, height, resolution, RGBA_CREAM_CAN);
    layers_[WEIGHTS_SOUTH_EAST] = createPixelLayer(WEIGHTS_SOUTH_EAST,
        width, height, resolution, RGBA_CREAM_CAN);
    layers_[WEIGHTS_SOUTH] = createPixelLayer(WEIGHTS_SOUTH,
        width, height, resolution, RGBA_CREAM_CAN);
    layers_[WEIGHTS_SOUTH_WEST] = createPixelLayer(WEIGHTS_SOUTH_WEST,
        width, height, resolution, RGBA_CREAM_CAN);
    layers_[WEIGHTS_WEST] = createPixelLayer(WEIGHTS_WEST,
        width, height, resolution, RGBA_CREAM_CAN);
    layers_[WEIGHTS_NORTH_WEST] = createPixelLayer(WEIGHTS_NORTH_WEST,
        width, height, resolution, RGBA_CREAM_CAN);

    // Go through all defined cells in the grid and
    // populate the layers
    WeightedGrid2d* grid = logicalMap_->getGrid();
    for (Location location : *grid)
    {
        // Store the obstacle pixels
        WeightedGrid2d::BaseType cost = grid->getPayload(location);
        if (cost == WeightedGrid2d::PAYLOAD_MAX)
        {
            layers_[OBSTACLES]->fillLocation(location);
        }

        // Store the cost area
        if (cost > 0)
        {
            layers_[COST_AREA]->fillLocation(location);
        }

        // Store the weights
        WeightedGrid2d::BaseType north;
        WeightedGrid2d::BaseType northEast;
        WeightedGrid2d::BaseType east;
        WeightedGrid2d::BaseType southEast;
        WeightedGrid2d::BaseType south;
        WeightedGrid2d::BaseType southWest;
        WeightedGrid2d::BaseType west;
        WeightedGrid2d::BaseType northWest;

        grid->getWeights(location, north, northEast,
            east, southEast,
            south, southWest,
            west, northWest);
        if (north > 0)
        {
            layers_[WEIGHTS_NORTH]->fillLocation(location, north);
        }
        if (northEast > 0)
        {
            layers_[WEIGHTS_NORTH_EAST]->fillLocation(location, northEast);
        }
        if (east > 0)
        {
            layers_[WEIGHTS_EAST]->fillLocation(location, east);
        }
        if (southEast > 0)
        {
            layers_[WEIGHTS_SOUTH_EAST]->fillLocation(location, southEast);
        }
        if (south > 0)
        {
            layers_[WEIGHTS_SOUTH]->fillLocation(location, south);
        }
        if (southWest > 0)
        {
            layers_[WEIGHTS_SOUTH_WEST]->fillLocation(location, southWest);
        }
        if (west > 0)
        {
            layers_[WEIGHTS_WEST]->fillLocation(location, west);
        }
        if (northWest > 0)
        {
            layers_[WEIGHTS_NORTH_WEST]->fillLocation(location, northWest);
        }
    }

    std::shared_ptr<AreaLayerDisplay> playSound = createAreaLayer(PLAY_SOUND,
        width, height, resolution, RGBA_SEA_GREEN);
    layers_[PLAY_SOUND] = playSound;

    std::shared_ptr<AreaLayerDisplay> queue = createAreaLayer(QUEUE,
        width, height, resolution, RGBA_DEEP_KOAMARU);
    layers_[QUEUE] = queue;

    std::shared_ptr<AreaLayerDisplay> setMaxVelocity = createAreaLayer(SET_MAX_VELOCITY,
        width, height, resolution, RGBA_DENIM);
    layers_[SET_MAX_VELOCITY] = setMaxVelocity;

    std::shared_ptr<AreaLayerDisplay> stayOnPath = createAreaLayer(STAY_ON_PATH,
        width, height, resolution, RGBA_VIOLET_RED);
    layers_[STAY_ON_PATH] = stayOnPath;

    for (auto labeledArea : logicalMap_->getAreas())
    {
        LogicalMap::LabeledArea area = labeledArea.second;

        // Search for PlaySound notes
        shared_ptr<NotePlaySound> notePlaySound =
            area.notes->get<NotePlaySound>(NotePlaySound::TYPE);
        if (notePlaySound)
        {
            playSound->addArea(area.label, area.surface, notePlaySound);
        }

        // Search for Queue notes
        shared_ptr<NoteQueue> noteQueue =
            area.notes->get<NoteQueue>(NoteQueue::TYPE);
        if (noteQueue)
        {
            queue->addArea(area.label, area.surface, noteQueue);
        }

        // Search for SetMaxVelocity notes
        shared_ptr<NoteSetMaxVelocity> noteSetMaxVelocity =
            area.notes->get<NoteSetMaxVelocity>(NoteSetMaxVelocity::TYPE);
        if (noteSetMaxVelocity)
        {
            setMaxVelocity->addArea(area.label, area.surface, noteSetMaxVelocity);
        }

        // Search for StayOnPath notes
        shared_ptr<NoteStayOnPath> noteStayOnPath =
            area.notes->get<NoteStayOnPath>(NoteStayOnPath::TYPE);
        if (noteStayOnPath)
        {
            stayOnPath->addArea(area.label, area.surface, noteStayOnPath);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::onInitialize()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::onEnable()
{
    if (!logicalMap_)
    {
        return;
    }

    for (auto layer : layers_)
    {
        layer->show(true);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapDisplay::onDisable()
{
    if (!logicalMap_)
    {
        return;
    }

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
std::shared_ptr<PixelLayerDisplay> LogicalMapDisplay::createPixelLayer(unsigned int order,
    unsigned int width, unsigned int height,
    double resolution, Ogre::RGBA color)
{
    std::shared_ptr<PixelLayerDisplay> layer = std::make_shared<PixelLayerDisplay>(order,
        width, height, resolution, color);
    layer->connectTo(scene_manager_, scene_node_);

    return layer;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<AreaLayerDisplay> LogicalMapDisplay::createAreaLayer(unsigned int order,
    unsigned int width, unsigned int height,
    double resolution, Ogre::RGBA color)
{
    std::shared_ptr<AreaLayerDisplay> layer = std::make_shared<AreaLayerDisplay>(order,
        width, height, resolution, color);
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
    if (!logicalMap_ || !isEnabled())
    {
        return;
    }

    layers_[BACKGROUND]->show(propertyLayerBackground_->getValue().toBool());
    layers_[COST_AREA]->show(propertyLayerCostArea_->getValue().toBool());
    layers_[OBSTACLES]->show(propertyLayerObstacles_->getValue().toBool());
    layers_[PLAY_SOUND]->show(propertyLayerPlaySound_->getValue().toBool());
    layers_[QUEUE]->show(propertyLayerQueue_->getValue().toBool());
    layers_[SET_MAX_VELOCITY]->show(propertyLayerSetMaxVelocity_->getValue().toBool());
    layers_[STAY_ON_PATH]->show(propertyLayerStayOnPath_->getValue().toBool());

    layers_[WEIGHTS_NORTH]->show(propertyLayerWeightsNorth_->getValue().toBool());
    layers_[WEIGHTS_NORTH_EAST]->show(propertyLayerWeightsNorthEast_->getValue().toBool());
    layers_[WEIGHTS_EAST]->show(propertyLayerWeightsEast_->getValue().toBool());
    layers_[WEIGHTS_SOUTH_EAST]->show(propertyLayerWeightsSouthEast_->getValue().toBool());
    layers_[WEIGHTS_SOUTH]->show(propertyLayerWeightsSouth_->getValue().toBool());
    layers_[WEIGHTS_SOUTH_WEST]->show(propertyLayerWeightsSouthWest_->getValue().toBool());
    layers_[WEIGHTS_WEST]->show(propertyLayerWeightsWest_->getValue().toBool());
    layers_[WEIGHTS_NORTH_WEST]->show(propertyLayerWeightsNorthWest_->getValue().toBool());
}

} // namespace srs

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(srs::LogicalMapDisplay, rviz::Display)

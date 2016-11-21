#include <srsnode_executive/LabeledAreasDetector.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace srs {

const Sound LabeledAreasDetector::SOUND_OFF = Sound(0, 3000, 250, 32, 0);
const Sound LabeledAreasDetector::WARNING_SOUND = Sound(100, 3000, 250, 32, 65000);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LabeledAreasDetector::LabeledAreasDetector() :
    srsMapStack_(nullptr)
{
    updateMapStack();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LabeledAreasDetector::evaluatePose(Pose<> robotPose)
{
    // Make sure that the map stack is the latest available
    updateMapStack();

    if (srsMapStack_)
    {
        LogicalMap::LabeledAreaMapType touchedAreas;
        logicalMap_->checkAreas(robotPose.x, robotPose.y, touchedAreas);

        // Determine which areas the robot is entering
        for (auto area : touchedAreas)
        {
            if (currentLabeledAreas_.find(area.second.label) == currentLabeledAreas_.end())
            {
                interpretArea(area.second, DirectionEnum::ENTERING);
            }
        }

        // Determine which areas the robot is exiting
        for (auto area : currentLabeledAreas_)
        {
            if (touchedAreas.find(area.second.label) == touchedAreas.end())
            {
                interpretArea(area.second, DirectionEnum::EXITING);
            }
        }

        currentLabeledAreas_ = touchedAreas;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void LabeledAreasDetector::interpretArea(LogicalMap::LabeledArea area, DirectionEnum direction)
{
    ROS_INFO_STREAM_COND_NAMED(direction == DirectionEnum::ENTERING,
        "executive", "Entering area " << area);
    ROS_INFO_STREAM_COND_NAMED(direction == DirectionEnum::EXITING,
        "executive", "Exiting area " << area);

    if (area.note.warning_sound())
    {
        channelSound_.publish(direction == DirectionEnum::ENTERING ? WARNING_SOUND : SOUND_OFF);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LabeledAreasDetector::updateMapStack()
{
    // Make sure that the neither the logical not the occupancy maps
    // have been re-published. In case, destroy what we have and
    // ask for a new stack
    if (tapMapStack_.newDataAvailable())
    {
        srsMapStack_ = tapMapStack_.pop();

        logicalMap_ = srsMapStack_->getLogicalMap();
    }
}


} // namespace srs



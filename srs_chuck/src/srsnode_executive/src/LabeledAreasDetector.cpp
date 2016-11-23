#include <srsnode_executive/LabeledAreasDetector.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace srs {

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

        // Determine which areas the robot is entering, and count
        // the requests for the different states
        for (auto area : touchedAreas)
        {
            if (currentLabeledAreas_.find(area.second.label) == currentLabeledAreas_.end())
            {
                interpretArea(area.second, DirectionEnum::ENTERING);
            }
        }

        // Determine which areas the robot is staying in, and count
        // the requests for the different states
        for (auto area : touchedAreas)
        {
            if (currentLabeledAreas_.find(area.second.label) != currentLabeledAreas_.end())
            {
                interpretArea(area.second, DirectionEnum::STAYING);
            }
        }

        // Determine which areas the robot is exiting, and count
        // the requests for the different states
        for (auto area : currentLabeledAreas_)
        {
            if (touchedAreas.find(area.second.label) == touchedAreas.end())
            {
                interpretArea(area.second, DirectionEnum::EXITING);
            }
        }

        executeRequests();

        currentLabeledAreas_ = touchedAreas;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LabeledAreasDetector::executeRequests()
{
    requestWarningSound_.execute();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void LabeledAreasDetector::interpretArea(LogicalMap::LabeledArea area, DirectionEnum direction)
{
    switch (direction)
    {
        case DirectionEnum::ENTERING:

            ROS_INFO_STREAM_NAMED("executive", "Entering area " << area);
            if (area.note.warning_sound())
            {
                requestWarningSound_.request();
            }

            break;

        case DirectionEnum::STAYING:

            ROS_DEBUG_STREAM_NAMED("executive", "Staying in area " << area);

            // Nothing to do at the moment

            break;

        case DirectionEnum::EXITING:

            ROS_INFO_STREAM_NAMED("executive", "Exiting area " << area);
            if (area.note.warning_sound())
            {
                requestWarningSound_.release();
            }

            break;
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



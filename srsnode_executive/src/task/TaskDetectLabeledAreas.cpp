#include <srsnode_executive/task/TaskDetectLabeledAreas.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void TaskDetectLabeledAreas::run(ExecutiveContext& context)
{
    Pose<> robotPose = context.robotPose;

    if (context.mapStack)
    {
        context.previousLabeledAreas = context.activeLabeledAreas;
        context.activeLabeledAreas.clear();

        LogicalMap::LabeledAreaMapType touchedAreas;
        context.mapStack->getLogicalMap()->checkAreas(robotPose.x, robotPose.y, touchedAreas);

        // Determine which areas the robot is entering, and count
        // the requests for the different states
        for (auto area : touchedAreas)
        {
            if (!doesExist(context.previousLabeledAreas, area.second.label))
            {
                context.activeLabeledAreas.push_back(ExecutiveContext::ActiveLabelType(
                    area.second, ExecutiveContext::DirectionEnum::ENTERING));
            }
        }

        // Determine which areas the robot is staying in, and count
        // the requests for the different states
        for (auto area : touchedAreas)
        {
            if (doesExist(context.previousLabeledAreas, area.second.label))
            {
                context.activeLabeledAreas.push_back(ExecutiveContext::ActiveLabelType(
                    area.second, ExecutiveContext::DirectionEnum::STAYING));
            }
        }

        // Determine which areas the robot is exiting, and count
        // the requests for the different states
        for (auto area : context.previousLabeledAreas)
        {
            if (touchedAreas.find(area.first.label) == touchedAreas.end() &&
                area.second != ExecutiveContext::DirectionEnum::EXITING)
            {
                context.activeLabeledAreas.push_back(ExecutiveContext::ActiveLabelType(
                    area.first, ExecutiveContext::DirectionEnum::EXITING));
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

bool TaskDetectLabeledAreas::doesExist(vector<ExecutiveContext::ActiveLabelType>& areaList, string label)
{
    for (auto area : areaList)
    {
        if (area.first.label == label)
        {
            return true;
        }
    }

    return false;
}

} // namespace srs



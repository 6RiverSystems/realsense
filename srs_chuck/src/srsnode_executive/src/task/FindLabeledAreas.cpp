#include <srsnode_executive/task/FindLabeledAreas.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
TreeNode<ExecutiveContext>::NodeResult FindLabeledAreas::execute(ExecutiveContext* context)
{
    if (context->mapStack)
    {
        LogicalMap::LabeledAreaMapType touchedAreas;
        context->mapStack->getLogicalMap()->checkAreas(
            context->robotPose.x, context->robotPose.y, touchedAreas);

        context->activeLabeledAreas.clear();

        // Determine which areas the robot is entering
        for (auto area : touchedAreas)
        {
            if (context->currentLabeledAreas.find(area.second.label) ==
                context->currentLabeledAreas.end())
            {
                context->activeLabeledAreas.push_back(ExecutiveContext::ActiveLabelType(
                        area.second,
                        ExecutiveContext::DirectionEnum::ENTERING));
            }
        }

        // Determine which areas the robot is exiting
        for (auto area : context->currentLabeledAreas)
        {
            if (touchedAreas.find(area.second.label) == touchedAreas.end())
            {
                context->activeLabeledAreas.push_back(ExecutiveContext::ActiveLabelType(
                        area.second,
                        ExecutiveContext::DirectionEnum::EXITING));
            }
        }

        // Determine in which areas the robot is staying
        for (auto area : context->currentLabeledAreas)
        {
            if (touchedAreas.find(area.second.label) != touchedAreas.end())
            {
                context->activeLabeledAreas.push_back(ExecutiveContext::ActiveLabelType(
                        area.second,
                        ExecutiveContext::DirectionEnum::STAYING));
            }
        }

        context->currentLabeledAreas = touchedAreas;
    }

    return SUCCEDED;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

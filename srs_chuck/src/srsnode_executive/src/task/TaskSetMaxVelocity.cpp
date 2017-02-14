#include <srsnode_executive/task/TaskSetMaxVelocity.hpp>

#include <srslib_framework/localization/map/mapnote/NoteSetMaxVelocity.hpp>
#include <srslib_framework/ros/function/service_call/ServiceCallConfig.hpp>
#include <srslib_framework/chuck/ChuckConfig.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void TaskSetMaxVelocity::run(ExecutiveContext& context)
{
    float newMaxVelocity = context.maxVelocity;
    bool velocityWasSet = false;

    for (auto detectedArea : context.activeLabeledAreas)
    {
        LogicalMap::LabeledArea area = detectedArea.first;
        shared_ptr<NoteSetMaxVelocity> note = area.notes->get<NoteSetMaxVelocity>(
            NoteSetMaxVelocity::TYPE);

        if (note)
        {
            float velocity = note->getMaxVelocity();

            switch (detectedArea.second)
            {
                case ExecutiveContext::DirectionEnum::ENTERING:

                    ROS_INFO_STREAM_NAMED("executive", "Entering SET_MAX_VELOCITY area " << area.label);

                    newMaxVelocity = !velocityWasSet ? velocity : min(newMaxVelocity, velocity);
                    velocityWasSet = true;

                    break;

                case ExecutiveContext::DirectionEnum::EXITING:

                    ROS_INFO_STREAM_NAMED("executive", "Exiting SET_MAX_VELOCITY area " << area.label);

                    newMaxVelocity = !velocityWasSet ? defaultMaxVelocity_ : newMaxVelocity;
                    velocityWasSet = true;

                    break;
            }
        }
    }

    if (velocityWasSet)
    {
        ROS_INFO_STREAM_NAMED("executive", "Setting velocity to " << newMaxVelocity << "m/s");

        ServiceCallConfig<double>::call(ChuckConfig::Entities::LOCAL_PLANNER,
            ChuckConfig::Parameters::MAX_VELOCITY, newMaxVelocity);
    }

    context.maxVelocity = newMaxVelocity;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

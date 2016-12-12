#include <srsnode_executive/task/TaskSetMaxVelocity.hpp>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <srslib_framework/localization/map/mapnote/NoteSetMaxVelocity.hpp>

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
        shared_ptr<NoteSetMaxVelocity> note = area.notes->get<NoteSetMaxVelocity>(MapNote::SET_MAX_VELOCITY);

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

        dynamic_reconfigure::ReconfigureRequest request;
        dynamic_reconfigure::Config configuration;

        dynamic_reconfigure::DoubleParameter doubleParameter;
        doubleParameter.name = "max_vel_x";
        doubleParameter.value = newMaxVelocity;
        configuration.doubles.push_back(doubleParameter);

        request.config = configuration;

        dynamic_reconfigure::ReconfigureResponse response;
        ros::service::call("/move_base/DWAPlannerROS/set_parameters", request, response);
    }

    context.maxVelocity = newMaxVelocity;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

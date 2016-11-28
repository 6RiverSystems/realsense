#include <srsnode_executive/task/TaskPlayWarningSound.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace srs {

const Sound TaskPlayWarningSound::SOUND_OFF = Sound(0, 3000, 250, 32, 0);
const Sound TaskPlayWarningSound::WARNING_SOUND = Sound(100, 3000, 250, 32, 65000);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void TaskPlayWarningSound::run(ExecutiveContext& context)
{
    for (auto detectedArea : context.activeLabeledAreas)
    {
        LogicalMap::LabeledArea area = detectedArea.first;

        switch (detectedArea.second)
        {
            case ExecutiveContext::DirectionEnum::ENTERING:

                ROS_INFO_STREAM_NAMED("executive", "Entering area " << area.label);
                if (area.note.warning_sound())
                {
                    context.resourceSound.request();
                }

                break;

            case ExecutiveContext::DirectionEnum::STAYING:

                ROS_DEBUG_STREAM_NAMED("executive", "Staying in area " << area.label);

                if (area.note.warning_sound())
                {
                    if (context.isRobotMoving)
                    {
                        context.resourceSound.request();
                    }
                    else
                    {
                        context.resourceSound.release();
                    }
                }

                break;

            case ExecutiveContext::DirectionEnum::EXITING:

                ROS_INFO_STREAM_NAMED("executive", "Exiting area " << area.label);
                if (area.note.warning_sound())
                {
                    context.resourceSound.release();
                }

                break;
        }
    }

    if (context.resourceSound.hasChanged())
    {
        channelSound_.publish(context.resourceSound.confirmed() ? WARNING_SOUND : SOUND_OFF);
    }

    context.resourceSound.freeze();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

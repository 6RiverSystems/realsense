#include <srsnode_executive/task/TaskPlaySound.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/localization/map/mapnote/NotePlaySound.hpp>

namespace srs {

const Sound TaskPlaySound::SOUND_OFF = Sound(0, 3000, 250, 32, 0);
const Sound TaskPlaySound::SOUND_WARNING = Sound(255, 3000, 500, 64, 65000);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void TaskPlaySound::run(ExecutiveContext& context)
{
    for (auto detectedArea : context.activeLabeledAreas)
    {
        LogicalMap::LabeledArea area = detectedArea.first;
        shared_ptr<NotePlaySound> note = area.notes->get<NotePlaySound>(NotePlaySound::TYPE);

        if (note)
        {
            switch (detectedArea.second)
            {
                case ExecutiveContext::DirectionEnum::ENTERING:

                    ROS_INFO_STREAM_NAMED("executive", "Entering PLAY_SOUND area " << area.label);
                    if (context.isRobotMoving && !context.isRobotPaused)
                    {
                        context.resourceSound.request();
                    }

                    break;

                case ExecutiveContext::DirectionEnum::STAYING:

                    ROS_DEBUG_STREAM_NAMED("executive", "Staying in PLAY_SOUND area " << area.label);

                    if (context.isRobotMoving && !context.isRobotPaused)
                    {
                        context.resourceSound.request();
                    }
                    else
                    {
                        context.resourceSound.release();
                    }

                    break;

                case ExecutiveContext::DirectionEnum::EXITING:

                    ROS_INFO_STREAM_NAMED("executive", "Exiting PLAY_SOUND area " << area.label);
                    if (context.isRobotMoving && !context.isRobotPaused)
                    {
                        context.resourceSound.release();
                    }

                    break;
            }
        }
    }

    if (context.resourceSound.hasChanged())
    {
        channelSound_.publish(context.resourceSound.confirmed() ? SOUND_WARNING : SOUND_OFF);
    }

    context.resourceSound.freeze();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

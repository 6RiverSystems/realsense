#include <srsnode_executive/task/TaskQueue.hpp>

#include <srslib_framework/localization/map/mapnote/NoteQueue.hpp>
#include <srslib_framework/ros/function/service_call/ServiceCallConfig.hpp>
#include <srslib_framework/chuck/ChuckConfig.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void TaskQueue::run(ExecutiveContext& context)
{
    for (auto detectedArea : context.activeLabeledAreas)
    {
        LogicalMap::LabeledArea area = detectedArea.first;
        shared_ptr<NoteQueue> note = area.notes->get<NoteQueue>(NoteQueue::TYPE);

        if (note)
        {
            switch (detectedArea.second)
            {
                case ExecutiveContext::DirectionEnum::ENTERING:

                    ROS_INFO_STREAM_NAMED("executive", "Entering QUEUE area " << area.label);

                    // As soon as the robot enters a queue, it has to switch to QUEUE mode.
                    // This is regardless of the goal is in a queue or not. A robot in a queue
                    // always has to behave, even if it's passing through
                    ServiceCallConfig<int>::set(ChuckConfig::Entities::LOCAL_PLANNER,
                        ChuckConfig::Parameters::LP_MODE, ChuckConfig::Parameters::LP_MODE_QUEUE);

                    break;

                case ExecutiveContext::DirectionEnum::STAYING:

                    // If the robot is currently in a queue but its goal is outside the queue,
                    // the LP mode can be switched to default
                    if (!context.goal.inQueue) {
                        ROS_INFO_STREAM_NAMED("executive", "Goal is not in QUEUE. Switching LP to DEFAULT mode");

                        ServiceCallConfig<int>::set(ChuckConfig::Entities::LOCAL_PLANNER,
                            ChuckConfig::Parameters::LP_MODE, ChuckConfig::Parameters::LP_MODE_DEFAULT);
                    }

                    break;

                case ExecutiveContext::DirectionEnum::EXITING:

                    ROS_INFO_STREAM_NAMED("executive", "Exiting QUEUE area " << area.label);

                    ServiceCallConfig<int>::set(ChuckConfig::Entities::LOCAL_PLANNER,
                        ChuckConfig::Parameters::LP_MODE, ChuckConfig::Parameters::LP_MODE_DEFAULT);

                    break;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

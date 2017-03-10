#include <srsnode_executive/task/TaskStayOnPath.hpp>

#include <srslib_framework/localization/map/mapnote/NoteStayOnPath.hpp>
#include <srslib_framework/ros/function/service_call/ServiceCallConfig.hpp>
#include <srslib_framework/chuck/ChuckConfig.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void TaskStayOnPath::run(ExecutiveContext& context)
{
    for (auto detectedArea : context.activeLabeledAreas)
    {
        LogicalMap::LabeledArea area = detectedArea.first;
        shared_ptr<NoteStayOnPath> note = area.notes->get<NoteQueue>(NoteStayOnPath::TYPE);

        if (note)
        {
            switch (detectedArea.second)
            {
                case ExecutiveContext::DirectionEnum::ENTERING:

                    ROS_INFO_STREAM_NAMED("executive", "Entering STAY-ON-PATH area " << area.label);

                    // As soon as the robot enters a queue, it has to switch to QUEUE mode.
                    // This is regardless of the goal is in a queue or not. A robot in a queue
                    // always has to behave, even if it's passing through
                    ServiceCallConfig<int>::set(ChuckConfig::Entities::LOCAL_PLANNER,
                        ChuckConfig::Parameters::LP_MODE, ChuckConfig::Parameters::LP_MODE_STAY_ON_PATH);

                    break;

                case ExecutiveContext::DirectionEnum::EXITING:

                    ROS_INFO_STREAM_NAMED("executive", "Exiting STAY-ON-PATH area " << area.label);

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

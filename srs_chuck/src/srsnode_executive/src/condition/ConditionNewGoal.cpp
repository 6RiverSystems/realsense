#include <srsnode_executive/condition/ConditionNewGoal.hpp>

#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/localization/map/mapnote/NoteQueue.hpp>

#include <ros/ros.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

ConditionNewGoal::ConditionNewGoal(Executive* owner) :
    SoftwareMessageHandler(owner)
{
    tapCurrentGoal_.attach(this);

    newGoalReceived_ = false;
    currentGoal_ = Pose<>::INVALID;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ConditionNewGoal::notified(Subscriber<geometry_msgs::PoseStamped>* subject)
{
    TapMoveBaseCurrentGoal* tapMoveBaseCurrentGoal = static_cast<TapMoveBaseCurrentGoal*>(subject);
    currentGoal_ = tapMoveBaseCurrentGoal->pop();
    newGoalReceived_ = true;

    ROS_INFO_STREAM("Received goal request " << currentGoal_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ConditionNewGoal::evaluate(ExecutiveContext& context)
{
    if (newGoalReceived_) {

        context.goal.goal = currentGoal_;
        context.goal.inQueue = false;
        context.goal.queue = "";

        // Find all the areas touched by the goal
        LogicalMap::LabeledAreaMapType touchedAreas;
        context.mapStack->getLogicalMap()->checkAreas(currentGoal_.x, currentGoal_.y, touchedAreas);

        // Verify that one of them is a queue
        for (auto detectedArea : touchedAreas)
        {
            LogicalMap::LabeledArea area = detectedArea.second;
            shared_ptr<NoteQueue> note = area.notes->get<NoteQueue>(NoteQueue::TYPE);

            if (note)
            {
                ROS_INFO_STREAM_NAMED("executive", "Goal in QUEUE area " <<
                    note->getValue() << " (" << area.label << ")");

                context.goal.inQueue = true;
                context.goal.queue = note->getValue();

                break;
            }
        }

        newGoalReceived_ = false;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

#include <srsnode_motion/MotionController.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MotionController::MotionController() :
    executionTime_(0.0),
    newCommandAvailable_(false),
    nextScheduledTime_(-1)
{
    nextScheduled_ = trajectory_.end();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::reset()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::run(double dT, Pose<> robotPose, Velocity<>* command)
{
    if (command)
    {
        executingCommand_ = Velocity<>(*command);
        newCommandAvailable_ = true;
    }

//    if (nextScheduledTime_ > -1)
//    {
//        ROS_DEBUG_STREAM("[" << executionTime_ << "] Waiting for " << nextScheduledTime_);
//        executionTime_ += dT;
//
//        if (executionTime_ >= nextScheduledTime_)
//        {
//            MilestoneType milestone = *nextScheduled_;
//
//            ROS_DEBUG_STREAM("Executing: " << milestone.first << " - " << milestone.second);
//
//            geometry_msgs::Twist message;
//
//            message.linear.x = milestone.second.linear;
//            message.linear.y = 0;
//            message.linear.z = 0;
//            message.angular.x = 0;
//            message.angular.y = 0;
//            message.angular.z = milestone.second.angular;
//
//            pubCmdVel_.publish(message);
//
//            nextScheduled_++;
//            nextScheduledTime_ = nextScheduled_ != trajectory_.end() ?
//                nextScheduled_->first.arrivalTime :
//                -1;
//        }
//    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

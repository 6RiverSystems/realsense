#include <srsnode_motion/MotionController.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MotionController::MotionController(unsigned int lookAhead) :
    currentTrajectory_(),
    executionTime_(0.0),
    executingCommand_(Velocity<>()),
    lookAhead_(lookAhead),
    newCommandAvailable_(false),
    nextScheduledTime_(-1),
    referencePose_(Pose<>())
{
    reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::reset()
{
    nextScheduledTime_ = -1;
    nextScheduled_ = currentTrajectory_.end();
    executionTime_ = 0.0;
    newCommandAvailable_ = false;
    executingCommand_ = Velocity<>();
    referencePose_ = Pose<>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::run(double dT, Pose<> robotPose)
{
    if (nextScheduledTime_ > -1)
    {
        ROS_DEBUG_STREAM("[" << executionTime_ << "] Waiting for " << nextScheduledTime_);
        executionTime_ += dT;

        if (executionTime_ >= nextScheduledTime_)
        {
            Trajectory::MilestoneType milestone = *nextScheduled_;
            Velocity<> prospectiveCommand = milestone.second;

            ROS_DEBUG_STREAM("Executing: " << milestone.first << " - " << prospectiveCommand);

            // If the two velocities are similar, there is no need to send a
            // new command. However the cursor of the milestone is advanced anyway.
            if (!similarVelocities(executingCommand_, prospectiveCommand))
            {
                executingCommand_ = prospectiveCommand;
                newCommandAvailable_ = true;
            }

            nextScheduled_++;
            if (nextScheduled_ != currentTrajectory_.end())
            {
                nextScheduledTime_ = nextScheduled_->first.arrivalTime;
            }
            else
            {
                // All done. The motion controller can be reset.
                reset();
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::setTrajectory(Trajectory::TrajectoryType& trajectory)
{
    currentTrajectory_ = trajectory;

    for (auto milestone : currentTrajectory_)
    {
        ROS_DEBUG_STREAM("Executing: " << milestone.first << " - " << milestone.second);
    }

    executionTime_ = 0.0;
    nextScheduled_ = currentTrajectory_.begin();
    nextScheduledTime_ = nextScheduled_->first.arrivalTime;

    //determineReferencePose();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::stop(double stopDistance)
{
    // TODO: Make the stop command function with the stop distance
    reset();

    newCommandAvailable_ = true;
    executingCommand_ = Velocity<>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

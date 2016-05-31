#include <srsnode_motion/MotionController.hpp>

#include <srslib_framework/math/PoseMath.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MotionController::MotionController(unsigned int lookAhead) :
    currentTrajectory_(),
    executionTime_(0.0),
    executingCommand_(Velocity<>()),
    goal_(Pose<>()),
    goalReached_(false),
    lookAhead_(50),  // (lookAhead) #############
    lowLevelController_(0.0, 0.9),
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
    referenceIndex_ = 0;

    goal_ = Pose<>();
    goalReached_ = false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::run(double dT, Pose<> robotPose)
{
    if (nextScheduledTime_ > -1)
    {
        ROS_INFO_STREAM("[" << executionTime_ << "] Waiting for " << nextScheduledTime_);
        executionTime_ += dT;

        if (executionTime_ >= nextScheduledTime_)
        {
            Trajectory::MilestoneType milestone = *nextScheduled_;
            Velocity<> prospectiveCommand = milestone.second;

            ROS_INFO_STREAM("Executing: " << milestone.first << " - " << prospectiveCommand);

            // If the two velocities are similar, there is no need to send a
            // new command. However the cursor of the milestone is advanced anyway.
            if (!similarVelocities(executingCommand_, prospectiveCommand))
            {
                executingCommand_ = prospectiveCommand;
                newCommandAvailable_ = true;
            }

            if (newCommandAvailable_)
            {
                // Pass the executing command to the low level controller
                // to modulate linear and angular velocity
                stepLLController(robotPose);
            }

            nextScheduled_++;
            if (nextScheduled_ != currentTrajectory_.end())
            {
                nextScheduledTime_ = executionTime_ + nextScheduled_->first.arrivalTime;
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

    if (!currentTrajectory_.empty())
    {
        int count = 0;
        for (auto milestone : currentTrajectory_)
        {
            ROS_INFO_STREAM(count++ << ": " << milestone.first << " - " << milestone.second);
        }

        if (isMoving())
        {
            stop(0.0);
        }
        reset();

        nextScheduled_ = currentTrajectory_.begin();
        nextScheduledTime_ = nextScheduled_->first.arrivalTime;

        goal_ = currentTrajectory_[currentTrajectory_.size() - 1].first;

        determineNextReferencePose();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::stop(double stopDistance)
{
    // TODO: Make the stop command function with the stop distance
    newCommandAvailable_ = true;
    executingCommand_ = Velocity<>();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

void MotionController::determineNextReferencePose()
{
    int trajectorySize = currentTrajectory_.size();

    referenceIndex_ += lookAhead_;
    if (referenceIndex_ > trajectorySize)
    {
        referenceIndex_ = trajectorySize - 1;
    }

    referencePose_ = currentTrajectory_[referenceIndex_].first;
    lowLevelController_.setReference(referencePose_);

    ROS_INFO_STREAM("Switching to reference pose [" << referenceIndex_ << "] " << referencePose_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::stepLLController(Pose<> robotPose)
{
    ROS_INFO_STREAM("Reported pose: " << robotPose);
    double d = PoseMath::euclidean(robotPose, goal_);

    ROS_INFO_STREAM("Distance to goal: " << d);
    if (d < 0.2)
    {
        goalReached_ = true;
        ROS_INFO_STREAM("Goal reached: " << goal_);
        stop(0.0);
    }
    else
    {
        d = PoseMath::euclidean(robotPose, referencePose_);
        ROS_INFO_STREAM("Distance to reference: " << d);

        if (d < 0.5)
        {
            determineNextReferencePose();
        }

        executingCommand_ = lowLevelController_.step(robotPose, executingCommand_);
    }
}

} // namespace srs

#include <srsnode_motion/MotionController.hpp>

#include <ros/ros.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::execute(Solution<Grid2d> solution)
{
    // If the first action of the solution is a rotation,
    // split the solution in two parts and give the rotation to
    // the appropriate controller

    solutions_.push(pair<int, Solution<Grid2d>>(FOLLOW, solution));

//    // TODO: Make sure that it can handle multiple rotations (is there a use case like that??)
//    SolutionNode<Grid2d> firstNode = solution.afterStart();
//    if (firstNode.actionType == SolutionNode<Grid2d>::ROTATE_180 ||
//        firstNode.actionType == SolutionNode<Grid2d>::ROTATE_M90 ||
//        firstNode.actionType == SolutionNode<Grid2d>::ROTATE_P90)
//    {
//        Solution<Grid2d> rotation;
//        Solution<Grid2d> path;
//        solution.split(1, rotation, path);
//
//        solutions_.push(pair<int, Solution<Grid2d>>(ROTATE, rotation));
//        solutions_.push(pair<int, Solution<Grid2d>>(FOLLOW, path));
//    }
//    else
//    {
//    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::reset()
{
    activeController_ = nullptr;

    currentOdometry_ = Odometry<>();
    currentState_ = StateEnum::NONE;
    currentTask_ = TaskEnum::STAND;

    emergencyDeclared_ = false;

    nextState_ = StateEnum::STANDING;
    nextTask_ = TaskEnum::STAND;

    pathFollower_->reset();

    while (!solutions_.empty())
    {
        solutions_.pop();
    }

    normalStop();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::run(Pose<> robotPose, Odometry<> odometry)
{
    currentPose_ = robotPose;
    currentOdometry_ = odometry;
    ROS_DEBUG_STREAM_NAMED("MotionController", "Reported robot pose: " << currentPose_);
    ROS_DEBUG_STREAM_NAMED("MotionController", "Reported odometry: " << currentOdometry_);

    // Handle the current state
    currentState_ = nextState_;
    switch (currentState_)
    {
        case StateEnum::FOLLOWING:
            ROS_DEBUG_STREAM_NAMED("MotionController", "State: FOLLOWING");
            stateFollowing();
            break;

        case StateEnum::STANDING:
            ROS_DEBUG_STREAM_NAMED("MotionController", "State: STANDING");
            stateStanding();
            break;

        case StateEnum::STOPPING:
            ROS_DEBUG_STREAM_NAMED("MotionController", "State: STOPPING");
            stateStopping();
            break;

        default:
            // TODO Handle unforeseen situation (NIL should never be seen here)
            break;
    }

    // If no emergency and there is some work in the queue to execute
    if (!emergencyDeclared_)
    {
        checkForWork();

        // If the task changed, execute the task entrance portion
        if (currentTask_ != nextTask_)
        {
            currentTask_ = nextTask_;
            switch (currentTask_)
            {
                case TaskEnum::EMERGENCY_STOP:
                    ROS_DEBUG_STREAM_NAMED("MotionController", "Requested task: EMERGENCY_STOP");
                    taskEmergencyStop();
                    break;

                case TaskEnum::FOLLOW:
                    ROS_DEBUG_STREAM_NAMED("MotionController", "Requested task: FOLLOW");
                    taskFollow();
                    break;

                case TaskEnum::NORMAL_STOP:
                    ROS_DEBUG_STREAM_NAMED("MotionController", "Requested task: NORMAL_STOP");
                    taskNormalStop();
                    break;

                case TaskEnum::ROTATE:
                    ROS_DEBUG_STREAM_NAMED("MotionController", "Requested task: ROTATE");
                    taskRotate();
                    break;

                case TaskEnum::STAND:
                    ROS_DEBUG_STREAM_NAMED("MotionController", "Requested task: STAND");
                    taskStand();
                    break;

                default:
                    // TODO Handle unforeseen situation (NIL should never be seen here)
                    break;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::setRobot(RobotProfile robot)
{
    robot_ = robot;

    pathFollower_->setGoalReachedDistance(robot_.goalReachedDistance);
    pathFollower_->setMaxLookAheadDistance(robot_.maxLookAheadDistance);
    pathFollower_->setMaxAngularVelocity(robot_.maxAngularVelocity);
    pathFollower_->setMaxLinearVelocity(robot_.maxLinearVelocity);
    pathFollower_->setMinLookAheadDistance(robot_.minLookAheadDistance);
    pathFollower_->setRatioLookAheadDistance(robot_.ratioLookAheadDistance);
    pathFollower_->setTravelAngularVelocity(robot_.travelAngularVelocity);
    pathFollower_->setTravelLinearVelocity(robot_.travelLinearVelocity);
    pathFollower_->setTravelRotationVelocity(robot_.travelRotationVelocity);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::checkForWork()
{
    // If there is work in the queue, change the task accordingly
    if (!solutions_.empty())
    {
        // First, find the work to do
        pair<int, Solution<Grid2d>> work = solutions_.front();
        solutions_.pop();

        // Schedule the next task and the solution
        TaskEnum task = static_cast<TaskEnum>(work.first);
        nextTask(task);
        nextSolution(work.second);

        switch (task)
        {
            case FOLLOW:
                activeController_ = pathFollower_;
                break;
        }
    }
    else
    {
        // Otherwise, just keep standing
        nextTask(STAND);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::stateFollowing()
{
    if (!activeController_ || (activeController_ && activeController_->isGoalReached()))
    {
        nextState(StateEnum::STANDING);
    }
    else
    {
        activeController_->stepController(currentPose_, currentOdometry_);

        nextState(StateEnum::FOLLOWING);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::stateStanding()
{
    // There is nothing to do regarding standing
    nextState(STANDING);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::stateStopping()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskEmergencyStop()
{
    emergencyDeclared_ = true;

    //executeCommand(Velocity<>());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskFollow()
{
    currentSolution_ = nextSolution_;

    Trajectory<> trajectory;
    TrajectoryGenerator converter(robot_);

    converter.fromSolution(currentSolution_, dT_);
    converter.getTrajectory(trajectory);

    pathFollower_->setTrajectory(trajectory, currentPose_);

    nextState(StateEnum::FOLLOWING);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskNormalStop()
{
    //executeCommand(Velocity<>());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskRotate()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskStand()
{
    // Nothing to do while standing
}

} // namespace srs

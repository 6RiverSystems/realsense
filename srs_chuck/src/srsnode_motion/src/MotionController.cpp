#include <srsnode_motion/MotionController.hpp>

#include <ros/ros.h>

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

    // TODO: Make sure that it can handle multiple rotations (is there a use case like that??)
    SolutionNode<Grid2d> firstNode = solution.afterStart();
    if (firstNode.actionType == SolutionNode<Grid2d>::ROTATE_180 ||
        firstNode.actionType == SolutionNode<Grid2d>::ROTATE_M90 ||
        firstNode.actionType == SolutionNode<Grid2d>::ROTATE_P90)
    {
//        Solution<Grid2d> rotation;
//        Solution<Grid2d> path;
//        solution.split(1, rotation, path);
//
//        solutions_.push(pair<int, Solution<Grid2d>>(ROTATE, rotation));
//        solutions_.push(pair<int, Solution<Grid2d>>(FOLLOW, path));
    }
    else
    {
        solutions_.push(pair<int, Solution<Grid2d>>(FOLLOW, solution));
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::reset()
{
    currentOdometry_ = Odometry<>();
    currentState_ = StateEnum::NONE;
    currentTask_ = TaskEnum::STAND;

    emergencyDeclared_ = false;
    executingCommand_ = Velocity<>();

    newCommandAvailable_ = false;
    nextState_ = StateEnum::STANDING;
    nextTask_ = TaskEnum::STAND;

    projectionIndex_ = -1;

    referencePose_ = Pose<>();
    referenceIndex_ = -1;

    goal_ = Pose<>();
    goalReached_ = false;

    while (!solutions_.empty())
    {
        solutions_.pop();
    }
    currentTrajectory_.clear();

    normalStop();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::run(Pose<> robotPose, Odometry<> odometry)
{
    currentRobotPose_ = robotPose;
    currentOdometry_ = odometry;
    ROS_INFO_STREAM_NAMED("MotionController", "Reported robot pose: " << currentRobotPose_);
    ROS_INFO_STREAM_NAMED("MotionController", "Reported odometry: " << currentOdometry_);

    // Handle the current state
    currentState_ = nextState_;
    switch (currentState_)
    {
        case StateEnum::FOLLOWING:
            ROS_INFO_STREAM_NAMED("MotionController", "State: FOLLOWING");
            stateFollowing();
            break;

        case StateEnum::STANDING:
            ROS_INFO_STREAM_NAMED("MotionController", "State: STANDING");
            stateStanding();
            break;

        case StateEnum::STOPPING:
            ROS_INFO_STREAM_NAMED("MotionController", "State: STOPPING");
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
                    ROS_INFO_STREAM_NAMED("MotionController", "Requested task: EMERGENCY_STOP");
                    taskEmergencyStop();
                    break;

                case TaskEnum::FOLLOW:
                    ROS_INFO_STREAM_NAMED("MotionController", "Requested task: FOLLOW");
                    taskFollow();
                    break;

                case TaskEnum::NORMAL_STOP:
                    ROS_INFO_STREAM_NAMED("MotionController", "Requested task: NORMAL_STOP");
                    taskNormalStop();
                    break;

                case TaskEnum::ROTATE:
                    ROS_INFO_STREAM_NAMED("MotionController", "Requested task: ROTATE");
                    taskRotate();
                    break;

                case TaskEnum::STAND:
                    ROS_INFO_STREAM_NAMED("MotionController", "Requested task: STAND");
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

    pathFollower_.setLookAheadDistance(robot_.lookAheadDistance);
    pathFollower_.setMaxAngularVelocity(robot_.maxAngularVelocity);
    pathFollower_.setMaxLinearVelocity(robot_.maxLinearVelocity);
    pathFollower_.setTravelAngularVelocity(robot_.travelAngularVelocity);
    pathFollower_.setTravelLinearVelocity(robot_.travelLinearVelocity);
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
        nextTask(static_cast<TaskEnum>(work.first));
        nextSolution(work.second);
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
    updateProjectionIndex();

    Velocity<> nextCommand;

    double distance = PoseMath::euclidean(currentRobotPose_, goal_);
    ROS_INFO_STREAM_NAMED("MotionController", "Distance to goal: " << distance);

    if (distance < robot_.goalReachedDistance)
    {
        goalReached_ = true;
        ROS_INFO_STREAM_NAMED("MotionController", "Goal reached: " << goal_);

        // Temporary fix to stop the robot when arriving to the goal
        // It must be removed when the velocities in the trajectory
        // are correct
        executeCommand(Velocity<>());

        nextState(StateEnum::STANDING);
    }
    else
    {
        distance = PoseMath::euclidean(currentRobotPose_, referencePose_);
        ROS_INFO_STREAM_NAMED("MotionController", "Distance to reference: " << distance);

        // Find the desired velocity command
        nextCommand = currentTrajectory_.getVelocity(projectionIndex_);

        // Ask the low-level controller to modulate the desired velocity
        nextCommand = pathFollower_.step(currentRobotPose_, nextCommand);

        // If the two velocities are similar, there is no need to send a
        // new command (if requested).
        if (!similarVelocities(executingCommand_, nextCommand))
        {
            executeCommand(nextCommand);
        }

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

    executeCommand(Velocity<>());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskFollow()
{
    Trajectory<> trajectory;

    TrajectoryGenerator converter(robot_);
    converter.fromSolution(nextSolution_);
    converter.getTrajectory(trajectory);

    cout << nextSolution_ << endl;
    cout << trajectory << endl;

    currentTrajectory_ = trajectory;

    if (!currentTrajectory_.empty())
    {
        currentSolution_ = nextSolution_;

        // Find the goal of the trajectory
        goal_ = currentTrajectory_.getGoal();

        // Find the first reference point on the given trajectory
        referenceIndex_ = currentTrajectory_.findClosestPose(currentRobotPose_);
        referencePose_ = currentTrajectory_.getPose(referenceIndex_);

        // Set the reference in the low-level controller
        pathFollower_.setReference(referencePose_);

        // Find the projection of the current robot pose onto the trajectory
        projectionIndex_ = currentTrajectory_.findClosestPose(currentRobotPose_,
            0, robot_.lookAheadDistance);
    }

    nextState(StateEnum::FOLLOWING);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskNormalStop()
{
    executeCommand(Velocity<>());
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

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::updateProjectionIndex()
{
    projectionIndex_ = currentTrajectory_.findClosestPose(currentRobotPose_,
        projectionIndex_, robot_.lookAheadDistance);
    referenceIndex_ = currentTrajectory_.findWaypointAtDistance(projectionIndex_,
        robot_.lookAheadDistance);

    referencePose_ = currentTrajectory_.getPose(referenceIndex_);
    pathFollower_.setReference(referencePose_);
}

} // namespace srs

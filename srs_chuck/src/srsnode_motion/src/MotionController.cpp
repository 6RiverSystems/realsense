#include <srsnode_motion/MotionController.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MotionController::MotionController(double dT) :
    dT_(dT),
    manualController_(nullptr),
    pathController_(nullptr),
    rosNodeHandle_()
{
    pubCmdVel_ = rosNodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    manualController_ = new ManualController();
    pathController_ = new CMUPathController();

    reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::emergencyStop()
{
    switchToManual();

    // Issue a zero velocity command immediately
    executeCommand(CommandEnum::E_STOP);

    nextTask(TaskEnum::EMERGENCY_STOP);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::execute(Solution<Grid2d> solution)
{
    // If the first action of the solution is a rotation,
    // split the solution in two parts and give the rotation to
    // the appropriate controller

    Solution<Grid2d>* newSolution = new Solution<Grid2d>(solution);
    pushWork(TaskEnum::PATH_FOLLOW, newSolution);

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
void MotionController::normalStop()
{
    pushWork(TaskEnum::NORMAL_STOP);
    switchToManual();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::reset()
{
    selectController(TaskEnum::NONE);

    currentOdometry_ = Odometry<>();
    currentState_ = StateEnum::NIL;
    currentTask_ = TaskEnum::STAND;

    emergencyDeclared_ = false;

    nextState_ = StateEnum::STANDING;
    nextTask_ = TaskEnum::STAND;

    pathController_->reset();
    manualController_->reset();

    while (!work_.empty())
    {
        work_.pop();
    }

    normalStop();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::run(Pose<> currentPose, Odometry<> currentOdometry, Velocity<> currentCommand)
{
    currentPose_ = currentPose;
    currentOdometry_ = currentOdometry;
    ROS_DEBUG_STREAM_NAMED("MotionController", "Reported robot pose: " << currentPose_);
    ROS_DEBUG_STREAM_NAMED("MotionController", "Reported odometry: " << currentOdometry_);

    // Handle the current state
    currentState_ = nextState_;
    switch (currentState_)
    {
        case StateEnum::RUNNING:
            ROS_DEBUG_STREAM_NAMED("MotionController", "State: RUNNING");
            stateRunning();
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
        // Look in the queue to see if there is work to do
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

                case TaskEnum::PATH_FOLLOW:
                    ROS_DEBUG_STREAM_NAMED("MotionController", "Requested task: PATH_FOLLOW");
                    taskPathFollow();
                    break;

                case TaskEnum::MANUAL_FOLLOW:
                    ROS_DEBUG_STREAM_NAMED("MotionController", "Requested task: MANUAL_FOLLOW");
                    taskManualFollow();
                    break;

                case TaskEnum::NONE:
                    ROS_DEBUG_STREAM_NAMED("MotionController", "Requested task: NIL");
                    // Nothing to do
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
            }
        }
    }

    if (activeController_)
    {
        // Make sure that the manual controller has the user command
        manualController_->setUserCommand(currentCommand);

        // Run the active controller and send the command
        // if something is available
        activeController_->step(currentPose_, currentOdometry_);
        if (activeController_->newCommandAvailable())
        {
            Velocity<> command = activeController_->getExecutingCommand();
            executeCommand(CommandEnum::VELOCITY, &command);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::setRobot(RobotProfile robot)
{
    robot_ = robot;

    pathController_->setGoalReachedDistance(robot_.goalReachedDistance);
    pathController_->setMaxLookAheadDistance(robot_.maxLookAheadDistance);
    pathController_->setMaxAngularVelocity(robot_.maxAngularVelocity);
    pathController_->setMaxLinearVelocity(robot_.maxLinearVelocity);
    pathController_->setMinLookAheadDistance(robot_.minLookAheadDistance);
    pathController_->setRatioLookAheadDistance(robot_.ratioLookAheadDistance);
    pathController_->setTravelAngularVelocity(robot_.travelAngularVelocity);
    pathController_->setTravelLinearVelocity(robot_.travelLinearVelocity);
    pathController_->setTravelRotationVelocity(robot_.travelRotationVelocity);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::switchToManual()
{
    if (activeController_ != manualController_)
    {
        pushWork(TaskEnum::MANUAL_FOLLOW);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::switchToAutonomous()
{
    if (activeController_ == manualController_)
    {
        pushWork(TaskEnum::NONE);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::executeCommand(CommandEnum command, Velocity<>* velocity)
{
    geometry_msgs::Twist messageTwist;

    switch (command)
    {
        case VELOCITY:
            messageTwist.linear.x = velocity->linear;
            messageTwist.linear.y = 0;
            messageTwist.linear.z = 0;
            messageTwist.angular.x = 0;
            messageTwist.angular.y = 0;
            messageTwist.angular.z = velocity->angular;

            ROS_DEBUG_STREAM_NAMED("MotionController", "Sending command: " << *velocity);
            pubCmdVel_.publish(messageTwist);
            break;

        case E_STOP:
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::checkForWork()
{
    // If there is work in the queue, change the task accordingly
    if (!work_.empty())
    {
        // First, find the work to do
        WorkType work = work_.front();
        work_.pop();

        // Schedule the next task
        TaskEnum task = static_cast<TaskEnum>(work.first);
        nextTask(task);

        // Specify the next solution (if any)
        if (work.second)
        {
            nextSolution_ = *work.second;
            delete work.second;
        }
        else
        {
            nextSolution_.clear();
        }

        selectController(task);
    }
    else
    {
        // If the controller is not moving the robot
        if (!isMoving())
        {
            // Otherwise, just keep standing
            nextTask(STAND);
        }

        // Otherwise, let the state machine complete whatever
        // work the controller was doing
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::selectController(TaskEnum task)
{
    activeController_ = nullptr;

    switch (task)
    {
        case PATH_FOLLOW:
            activeController_ = pathController_;
            break;

        case MANUAL_FOLLOW:
            activeController_ = manualController_;
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::stateRunning()
{
    if (!activeController_ || (activeController_ && activeController_->isGoalReached()))
    {
        nextState(StateEnum::STANDING);
    }
    else
    {
        nextState(StateEnum::RUNNING);
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
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskManualFollow()
{
    // Change state
    nextState(StateEnum::RUNNING);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskNormalStop()
{
    //executeCommand(Velocity<>());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskPathFollow()
{
    // Switch the current solution to the one that needs to be executed
    currentSolution_ = nextSolution_;

    // Calculate the trajectory
    Trajectory<> trajectory;
    TrajectoryGenerator converter(robot_);

    converter.fromSolution(currentSolution_, dT_);
    converter.getTrajectory(trajectory);

    // Pass the trajectory to the path controller
    pathController_->setTrajectory(trajectory, currentPose_);

    // Change state
    nextState(StateEnum::RUNNING);
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

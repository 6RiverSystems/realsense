#include <srsnode_motion/MotionController.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>

namespace srs {

const Velocity<> MotionController::ZERO_VELOCITY = Velocity<>(0.0, 0.0);

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
    rotationController_ = new RotationController();

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
    // Create a copy of the specified solution
    Solution<Grid2d>* pathSolution = new Solution<Grid2d>(solution);

    // Remove the START and GOAL nodes from the solution
    pathSolution->erase(pathSolution->begin());
    pathSolution->erase(pathSolution->end() - 1);

    // TODO: Make sure that it can handle multiple rotations.
    // Is there a use case like that?: In theory A* can generate a solution
    // with multiple rotations at the beginning to avoid the costly 180 rotation

    Solution<Grid2d>* initialRotationSolution = nullptr;
    Solution<Grid2d>* finalRotationSolution = nullptr;

    // Check if the solution starts with a rotation. If that is the case
    // replace it with rotation that takes in account the current pose
    // of the robot
    SolutionNode<Grid2d> startNode = pathSolution->getStart();
    if (startNode.actionType == SolutionNode<Grid2d>::ROTATE)
    {
        startNode.fromPose = currentPose_;
        initialRotationSolution = new Solution<Grid2d>(startNode);

        // Remove the rotation from the original solution
        pathSolution->erase(pathSolution->begin());
    }

    // Check if the solution ends with a rotation. If that is the case
    // replace it with controlled rotation
    SolutionNode<Grid2d> goalNode = pathSolution->getGoal();
    if (goalNode.actionType == SolutionNode<Grid2d>::ROTATE)
    {
        finalRotationSolution = new Solution<Grid2d>(goalNode);

        // Remove the rotation from the original solution
        pathSolution->erase(pathSolution->end() - 1);
    }

    // Push the solutions into the work queue
    if (initialRotationSolution)
    {
        pushWork(TaskEnum::ROTATE, initialRotationSolution);
    }
    pushWork(TaskEnum::PATH_FOLLOW, pathSolution);
    if (finalRotationSolution)
    {
        pushWork(TaskEnum::ROTATE, finalRotationSolution);
    }
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
    // If the robot was moving for some reason,
    // stop it immediately
    executeCommand(CommandEnum::VELOCITY, &ZERO_VELOCITY);

    // Reset all the controllers
    pathController_->reset();
    manualController_->reset();
    rotationController_->reset();

    // No controller is active
    selectController(TaskEnum::NONE);

    currentOdometry_ = Odometry<>();
    currentState_ = StateEnum::NIL;
    currentTask_ = TaskEnum::STAND;

    emergencyDeclared_ = false;

    nextState_ = StateEnum::STANDING;
    nextTask_ = TaskEnum::STAND;

    // Remove any work in the queue
    while (!work_.empty())
    {
        work_.pop();
    }
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

            // Select the appropriate controller for the task
            selectController(currentTask_);

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

    pathController_->setRobotProfile(robot);
    manualController_->setRobotProfile(robot);
    rotationController_->setRobotProfile(robot);
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
void MotionController::executeCommand(CommandEnum command, const Velocity<>* velocity)
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
    // Grab more work only if the motion controller
    // is not moving the robot
    if (!isMoving())
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
                cout << "--------------------------------------" << endl;
                cout << nextSolution_ << endl;
                cout << "--------------------------------------" << endl;

                delete work.second;
            }
            else
            {
                nextSolution_.clear();
            }
        }
        else
        {
            // If the controller is not moving the robot
            nextTask(TaskEnum::STAND);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::selectController(TaskEnum task)
{
    switch (task)
    {
        case MANUAL_FOLLOW:
            activeController_ = manualController_;
            break;

        case NORMAL_STOP:
            // TODO: Normal stop should have its own controller. For now
            // there is nothing else to do
            activeController_ = nullptr;
            break;

        case PATH_FOLLOW:
            activeController_ = pathController_;
            break;

        case ROTATE:
            activeController_ = rotationController_;
            break;

        case STAND:
            // TODO: Stand should have its own controller in case the robot
            // is pushed out of the way

        case NONE:
            activeController_ = nullptr;
            break;

        default:
            // In all other cases, there might be a problem
            // or the implementation of the case is missing
            throw;
    }

    // Before using the new controller, make sure that
    // it starts from a good state
    if (activeController_)
    {
        activeController_->reset();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::stateRunning()
{
    if (!activeController_ || (activeController_ && activeController_->isGoalReached()))
    {
        nextState(StateEnum::STANDING);
        nextTask(TaskEnum::STAND);
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
    // Switch the current solution to the one that needs to be executed
    currentSolution_ = nextSolution_;

    // Pass the goal to the path controller
    Pose<> goalPose = currentSolution_.getGoal().toPose;
    rotationController_->setGoal(goalPose);

    // Change state
    nextState(StateEnum::RUNNING);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskStand()
{
    // Nothing to do while standing
}

} // namespace srs

#include <srsnode_motion/MotionController.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/SolutionGenerator.hpp>

namespace srs {

const Velocity<> MotionController::ZERO_VELOCITY = Velocity<>(0.0, 0.0);

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MotionController::MotionController(double dT) :
    dT_(dT),
    rosNodeHandle_()
{
    pubCmdVel_ = rosNodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);

    manualController_ = new ManualController();
    pathController_ = new CMUPathController();
    rotationController_ = new RotationController();
    stopController_ = new StopController();
    standController_ = new StandController();

    reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::emergencyStop()
{
    switchToManual();

    // Issue a zero velocity command immediately
    executeCommand(true, CommandEnum::E_STOP);

    cleanWorkQueue();
    pushWork(TaskEnum::EMERGENCY_STOP);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::execute(Solution<Grid2d> solution)
{
//    // If the robot is already moving, cancel the previous work
//    // and execute what has been asked
//    if (isMoving())
//    {
//        // Clean the pending work
//        cleanWorkQueue();
//        pushWork(TaskEnum::NORMAL_STOP);
//
//        // Cancel the work in the controller
//        activeController_->cancel();
//    }

    // Create a copy of the specified solution
    Solution<Grid2d>* pathSolution = new Solution<Grid2d>(solution);

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
    currentCommand_ = ZERO_VELOCITY;
    executeCommand(true, CommandEnum::VELOCITY, &ZERO_VELOCITY);

    // Reset all the controllers
    pathController_->reset();
    manualController_->reset();
    rotationController_->reset();
    stopController_->reset();
    standController_->reset();

    // No controller is active
    currentTask_ = TaskEnum::NONE;
    selectController(TaskEnum::STAND);

    currentOdometry_ = Odometry<>();

    emergencyDeclared_ = false;

    cleanWorkQueue();
    pushWork(TaskEnum::STAND);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::run(Pose<> currentPose, Odometry<> currentOdometry, Velocity<> currentCommand)
{
    currentPose_ = currentPose;
    ROS_DEBUG_STREAM_NAMED("MotionController", "Reported robot pose: " << currentPose_);

    currentOdometry_ = currentOdometry;
    ROS_DEBUG_STREAM_NAMED("MotionController", "Reported odometry: " << currentOdometry_);

    checkMotionStatus();

    // Make sure that the manual controller has the user command
    manualController_->setUserCommand(currentCommand);

    // Run the active controller and send the command
    // if something is available
    activeController_->step(dT_, currentPose_, currentOdometry_);

    // Send the command for execution. Similar commands will not be sent
    Velocity<> command = activeController_->getExecutingCommand();
    executeCommand(false, CommandEnum::VELOCITY, &command);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::setRobot(RobotProfile robot)
{
    robot_ = robot;

    pathController_->setRobotProfile(robot);
    manualController_->setRobotProfile(robot);
    rotationController_->setRobotProfile(robot);
    standController_->setRobotProfile(robot);
    stopController_->setRobotProfile(robot);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::switchToManual()
{
    if (!isManualControllerActive())
    {
        // Cancel the current activity and clear the work queue
        activeController_->cancel();
        cleanWorkQueue();

        pushWork(TaskEnum::NORMAL_STOP);
        pushWork(TaskEnum::MANUAL_FOLLOW);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::switchToAutonomous()
{
    if (isManualControllerActive())
    {
        // Cancel the current activity
        activeController_->cancel();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::executeCommand(bool enforce, CommandEnum command, const Velocity<>* velocity)
{
    geometry_msgs::Twist messageTwist;

    switch (command)
    {
        case VELOCITY:
            if (enforce || !VelocityMath::equal(currentCommand_, *velocity))
            {
                currentCommand_ = *velocity;

                messageTwist.linear.x = currentCommand_.linear;
                messageTwist.linear.y = 0;
                messageTwist.linear.z = 0;
                messageTwist.angular.x = 0;
                messageTwist.angular.y = 0;
                messageTwist.angular.z = currentCommand_.angular;

                ROS_INFO_STREAM_NAMED("MotionController", "Sending velocity: " << currentCommand_);
                pubCmdVel_.publish(messageTwist);
            }
            break;

        case E_STOP:
            // For now the E-STOP is implemented with a 0 velocity. However
            // this can be changed to send a specific command to the brainstem
            currentCommand_ = Velocity<>();

            messageTwist.linear.x = currentCommand_.linear;
            messageTwist.linear.y = 0;
            messageTwist.linear.z = 0;
            messageTwist.angular.x = 0;
            messageTwist.angular.y = 0;
            messageTwist.angular.z = currentCommand_.angular;

            ROS_INFO_STREAM_NAMED("MotionController", "Sending e-stop: " << currentCommand_);
            pubCmdVel_.publish(messageTwist);
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::checkMotionStatus()
{
    // If the stand controller is currently active and there is work in
    // the queue, cancel work in the controller and pump work from the queue
    if (isStandControllerActive() && isWorkPending())
    {
        activeController_->cancel();
    }

    // If the controller says that the goal has been reached, and
    // the work has not been canceled for some reason
    if (activeController_->isGoalReached())
    {
        // If the active controller is the path controller, it is not
        // guaranteed that the final angle of the robot matches the angle
        // of the goal. In that case, a rotation is added to complete the movement
        if (isPathControllerActive() && !activeController_->isCanceled())
        {
            Pose<> goal = activeController_->getGoal();

            // If the current angle of the robot is not what was asked
            if (!AngleMath::equalRad<double>(goal.theta, currentPose_.theta,
                robot_.goalReachedAngle))
            {
                Solution<Grid2d>* solution = SolutionGenerator<Grid2d>::fromRotation(
                    currentPose_,
                    currentPose_.theta, goal.theta);

                pushWork(TaskEnum::ROTATE, solution);

                return;
            }
        }

        // At this point the previous task is completed

        // If no emergency has been declared
        if (!emergencyDeclared_)
        {
            // If there is nothing else to do, simply stand still
            if (!isWorkPending())
            {
                pushWork(TaskEnum::STAND);
            }

            // Look in the queue to see if there is work to do
            pumpWorkFromQueue();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::cleanWorkQueue()
{
    // Remove any work in the queue
    while (isWorkPending())
    {
        WorkType work = work_.front();
        work_.pop();

        // Delete the solution associated with the work
        if (work.second)
        {
            delete work.second;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::pumpWorkFromQueue()
{
    // First, find the work to do
    WorkType work = work_.front();
    work_.pop();

    // Schedule the next task
    currentTask_ = static_cast<TaskEnum>(work.first);

    // Specify the next solution (if any)
    if (work.second)
    {
        currentSolution_ = *work.second;
        delete work.second;
    }
    else
    {
        currentSolution_.clear();
    }

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
            ROS_DEBUG_STREAM_NAMED("MotionController", "Requested task: NONE");
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

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::selectController(TaskEnum task)
{
    switch (task)
    {
        case EMERGENCY_STOP:
            activeController_ = stopController_;
            break;

        case MANUAL_FOLLOW:
            activeController_ = manualController_;
            break;

        case NORMAL_STOP:
            activeController_ = stopController_;
            break;

        case PATH_FOLLOW:
            activeController_ = pathController_;
            break;

        case ROTATE:
            activeController_ = rotationController_;
            break;

        case STAND:
            activeController_ = standController_;
            break;

        default:
            throw;
    }

    // Before using the new controller, make sure that
    // it starts from a good state
    activeController_->reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskEmergencyStop()
{
    emergencyDeclared_ = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskManualFollow()
{
    // Nothing to do but follow the manual commands
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskNormalStop()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskPathFollow()
{
    // Calculate the trajectory
    Trajectory<> trajectory;
    TrajectoryGenerator converter(robot_);

    converter.fromSolution(currentSolution_, dT_);
    converter.getTrajectory(trajectory);

    // Pass the trajectory to the path controller
    pathController_->setTrajectory(trajectory, currentPose_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskRotate()
{
    // Pass the goal to the path controller
    Pose<> goalPose = currentSolution_.getGoal().toPose;
    rotationController_->setGoal(goalPose);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskStand()
{
    // Nothing to do while standing
}

} // namespace srs

#include <srsnode_motion/MotionController.hpp>

#include <iostream>
#include <iomanip>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>
#include <srslib_framework/planning/pathplanning/TrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/SolutionGenerator.hpp>

namespace srs {

const Velocity<> MotionController::ZERO_VELOCITY = Velocity<>(0.0, 0.0);

unordered_map<int, string> MotionController::TASK_NAMES = {
    {TaskEnum::EMERGENCY_STOP, "EMERGENCY_STOP"},
    {TaskEnum::MANUAL_FOLLOW, "MANUAL_FOLLOW"},
    {TaskEnum::NONE, "NONE"},
    {TaskEnum::NORMAL_STOP, "NORMAL_STOP"},
    {TaskEnum::PATH_FOLLOW, "PATH_FOLLOW"},
    {TaskEnum::ROTATE, "ROTATE"},
    {TaskEnum::STAND, "STAND"}
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
MotionController::MotionController(double dT) :
    dT_(dT),
    hasArrived_(false),
    hasArrivedChanged_(false),
    rosNodeHandle_()
{
    pubCmdVel_ = rosNodeHandle_.advertise<geometry_msgs::Twist>(
        "/internal/drivers/brainstem/cmd_velocity", 100);

    manualController_ = new ManualController();
    pathController_ = new CMUPathController();
    rotationController_ = new RotationController();
    stopController_ = new StopController();
    standController_ = new StandController();
    emergencyController_ = new EmergencyController();

    reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::emergencyStop()
{
    // Issue a zero velocity command immediately
    executeCommand(true, CommandEnum::E_STOP);

    // Cancel the current activity and clear the work queue
    activeController_->cancel();
    cleanWorkQueue();

    pushWork(TaskEnum::EMERGENCY_STOP);

    // Pump work from queue immediately, do not wait for the next cycle
    pumpWorkFromQueue();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::execute(Solution<Grid2d> solution)
{
    if (isEmergencyDeclared())
    {
        // Do not add anything to the work queue if
        // an emergency is in progress
        return;
    }

    // If the robot is already moving, cancel the previous work, stop,
    // and execute what has been asked
    if (!isStandControllerActive())
    {
        // Clean the pending work
        cleanWorkQueue();
        pushWork(TaskEnum::NORMAL_STOP);

        // Cancel the work in the controller
        activeController_->cancel();
    }

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
    currentFinalGoal_ = pathSolution->getGoal().toPose;

    if (finalRotationSolution)
    {
        pushWork(TaskEnum::ROTATE, finalRotationSolution);
        currentFinalGoal_ = finalRotationSolution->getGoal().toPose;
    }

    setHasArrived(false);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::normalStop()
{
    // Ignore the normal stop request if an
    // emergency has been declared
    if (isEmergencyDeclared())
    {
        return;
    }

    // Cancel the current activity and clear the work queue
    activeController_->cancel();
    cleanWorkQueue();

    pushWork(TaskEnum::NORMAL_STOP);
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
    emergencyController_->reset();

    // No controller is active
    currentTask_ = TaskEnum::NONE;
    selectController(TaskEnum::STAND);

    currentOdometry_ = Odometry<>();

    hasArrived_ = false;
    hasArrivedChanged_ = false;

    cleanWorkQueue();
    pushWork(TaskEnum::STAND);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::run(Pose<> currentPose, Odometry<> currentOdometry, Velocity<> currentCommand)
{
    hasArrivedChanged_ = false;

    currentPose_ = currentPose;
    ROS_DEBUG_STREAM_NAMED("MotionController", "Reported robot pose: " << currentPose_);

    currentOdometry_ = currentOdometry;
    ROS_DEBUG_STREAM_NAMED("MotionController", "Reported odometry: " << currentOdometry_);

    checkMotionStatus();

    // Make sure that the manual controller has the user command
    manualController_->setUserCommand(currentCommand);
    emergencyController_->setUserCommand(currentCommand);

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
    emergencyController_->setRobotProfile(robot);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::switchToManual()
{
    // Ignore the request request if an
    // emergency has been declared
    if (isEmergencyDeclared())
    {
        return;
    }

    if (!isManualControllerActive())
    {
        // Cancel the current activity and clear the work queue
        activeController_->cancel();
        cleanWorkQueue();

        pushWork(TaskEnum::MANUAL_FOLLOW);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::switchToAutonomous()
{
    // Ignore the request request if an
    // emergency has been declared
    if (isEmergencyDeclared())
    {
        return;
    }

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

            ROS_INFO_STREAM_NAMED("MotionController", "Sending e-stop");
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
        ROS_INFO_NAMED("MotionController", "Abandoned STAND");
        activeController_->cancel();
    }

    // If the controller says that the goal has been reached, and
    // the work has not been canceled for some reason
    if (activeController_->isGoalReached())
    {
        if (activeController_->isCanceled())
        {
            ROS_INFO_STREAM_NAMED("MotionController", "Controller " <<
                activeController_->getName() << " goal canceled");
        }
        else
        {
            ROS_INFO_STREAM_NAMED("MotionController", "Controller " <<
                activeController_->getName() << " reached its goal");
        }

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
                ROS_INFO_STREAM_NAMED("MotionController", "Rotation needed from: " <<
                    AngleMath::rad2deg<double>(currentPose_.theta) << "deg to: " <<
                    AngleMath::rad2deg<double>(goal.theta) << "deg");

                Solution<Grid2d>* solution = SolutionGenerator<Grid2d>::fromRotation(
                    currentPose_,
                    currentPose_.theta, goal.theta);

                pushWork(TaskEnum::ROTATE, solution);
            }
        }

        // If the robot was using moving controllers, but everything has
        // been completed, declare that the requested goal was reached
        if (isMovingControllerActive() && !isWorkPending())
        {
            setHasArrived(true);
        }

        // If no emergency has been declared, consider more work
        if (!isEmergencyDeclared())
        {
            // If there is nothing else to do, simply stand still
            if (!isWorkPending())
            {
                ROS_INFO_NAMED("MotionController", "No work was found");
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
        work_.pop_front();

        // Delete the solution associated with the work
        if (work.second)
        {
            delete work.second;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
string MotionController::printWorkToString()
{
    ostringstream output;

    output << "WORK QUEUE" << endl;
    int counter = 0;
    for (auto workTask : work_)
    {
        output << setw(3) << counter++ << ": " << TASK_NAMES[workTask.first]<< endl;
        if (workTask.second)
        {
            output << *workTask.second << endl;
        }
        else
        {
            output << "null" << endl;
        }
    }

    return output.str();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::pumpWorkFromQueue()
{
    // First, find the work to do
    WorkType work = work_.front();
    work_.pop_front();

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
        ROS_INFO_STREAM_NAMED("MotionController", "Requested task: " << TASK_NAMES[currentTask_]);

        case TaskEnum::EMERGENCY_STOP:
            taskEmergencyStop();
            break;

        case TaskEnum::PATH_FOLLOW:
            taskPathFollow();
            break;

        case TaskEnum::MANUAL_FOLLOW:
            taskManualFollow();
            break;

        case TaskEnum::NORMAL_STOP:
            taskNormalStop();
            break;

        case TaskEnum::ROTATE:
            taskRotate();
            break;

        case TaskEnum::STAND:
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
            activeController_ = emergencyController_;
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

    ROS_INFO_STREAM_NAMED("MotionController", "Switching to controller " <<
        activeController_->getName());

    // Before using the new controller, make sure that
    // it starts from a good state
    activeController_->reset();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskEmergencyStop()
{
    // Nothing to do but follow the manual commands
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskManualFollow()
{
    // Nothing to do but follow the manual commands
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskNormalStop()
{
    // Nothing to do while stopping
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskPathFollow()
{
    // Store what goal the controller is going to work on
    currentShortTermGoal_ = currentSolution_.getGoal().toPose;

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
    // Store what goal the controller is going to work on
    currentShortTermGoal_ = currentSolution_.getGoal().toPose;

    // Pass the goal to the path controller
    rotationController_->setGoal(currentShortTermGoal_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::taskStand()
{
    // Nothing to do while standing
}

} // namespace srs

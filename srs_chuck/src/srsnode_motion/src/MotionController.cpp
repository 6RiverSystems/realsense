#include <srsnode_motion/MotionController.hpp>

#include <iostream>
#include <iomanip>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>

#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>

#include <srsnode_motion/FactoryRobotProfile.hpp>

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
    hasArrived_(true),
    hasArrivedChanged_(false),
    rosNodeHandle_()
{
    pubCmdVel_ = rosNodeHandle_.advertise<geometry_msgs::Twist>(
        "/internal/drivers/brainstem/cmd_velocity", 100);

    pubLookAheadDistance_ = rosNodeHandle_.advertise<std_msgs::Float64>(
        "/internal/motion/look_ahead_distance", 1);

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
    executeCommand(true, CommandEnum::BRAKE);

    // Cancel the current activity and clear the work queue
    activeController_->cancel();
    cleanWorkQueue();

    pushWorkItem(TaskEnum::EMERGENCY_STOP);

    // Pump work from queue immediately, do not wait for the next cycle
    pumpWorkFromQueue();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::execute(SolutionType solution)
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
        pushWorkItem(TaskEnum::NORMAL_STOP);

        // Cancel the work in the controller
        activeController_->cancel();
    }

    if (!solution.empty())
    {
        // Decompose the solution in work items and
        // push them in the queue
        pushWorkSolution(solution);
    }
    else
    {
        pushWorkItem(TaskEnum::PATH_FOLLOW, nullptr);
        currentFinalGoal_ = currentPose_;
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

    pushWorkItem(TaskEnum::NORMAL_STOP);
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

    hasArrived_ = true;
    hasArrivedChanged_ = false;

    cleanWorkQueue();
    pushWorkItem(TaskEnum::STAND);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::run(Pose<> currentPose, Odometry<> currentOdometry, Velocity<> currentCommand)
{
    hasArrivedChanged_ = false;

    currentPose_ = currentPose;
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "MotionController",
        "Reported robot pose: " << currentPose_);

    currentOdometry_ = currentOdometry;
    ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "MotionController",
        "Reported odometry: " << currentOdometry_);

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

    publishLookAheadDistance();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::setConfiguration(MotionConfig& configuration)
{
    RobotProfile robot = FactoryRobotProfile::fromConfiguration(configuration);

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

    // If manual follow was not scheduled
    if (!isScheduled(TaskEnum::MANUAL_FOLLOW) && !isManualControllerActive())
    {
        // Cancel the current activity and clear the work queue
        activeController_->cancel();
        cleanWorkQueue();

        // Schedule a normal stop followed by a manual follow
        pushWorkItem(TaskEnum::NORMAL_STOP);
        pushWorkItem(TaskEnum::MANUAL_FOLLOW);
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
        // Cancel the current manual controller activity
        activeController_->cancel();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::aggregateInitialRotations(Pose<> pose,
    SolutionType*& solution, SolutionType*& rotation)
{
    // Do nothing if there is nothing in the solution to process
    if (solution->empty())
    {
        return;
    }

    // Check if the solution starts with a rotation. If that is the case
    // replace it with rotation that takes in account the specified initial pose
    GridSolutionItem node = solution->getStart();
    if (node.actionType != GridSolutionItem::ROTATE)
    {
        return;
    }

    Pose<> fromPose = Pose<>(node.fromPose.x, node.fromPose.y, pose.theta);

    // Remove all the ROTATE nodes at the beginning of the solution. Check
    // if the solution is empty while doing that in case it was composed exclusively by
    // ROTATE commands
    while (node.actionType == GridSolutionItem::ROTATE && !solution->empty())
    {
        // Remove the rotation from the original solution
        solution->erase(solution->begin());

        // Try the next solution node
        node = solution->getStart();
    }

    double theta = node.toPose.theta;
    Pose<> toPose = Pose<>(fromPose.x, fromPose.y, AngleMath::normalizeAngleRad<double>(theta));

    node = GridSolutionItem(GridSolutionItem::ROTATE, fromPose, toPose);
    rotation = new SolutionType(node);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::aggregateFinalRotations(SolutionType*& solution, SolutionType*& rotation)
{
    // Do nothing if there is nothing in the solution to process
    if (solution->empty())
    {
        return;
    }

    // Check if the solution ends with a rotation
    GridSolutionItem node = solution->getGoal();
    if (node.actionType != GridSolutionItem::ROTATE)
    {
        return;
    }

    Pose<> toPose = node.toPose;

    // Remove all the ROTATE nodes at the end of the solution
    while (node.actionType == GridSolutionItem::ROTATE)
    {
        // Remove the rotation from the original solution
        solution->erase(solution->end() - 1);

        // Try the next solution node
        node = solution->getGoal();
    }

    double theta = node.fromPose.theta;
    Pose<> fromPose = Pose<>(toPose.x, toPose.y, AngleMath::normalizeAngleRad<double>(theta));

    node = GridSolutionItem(GridSolutionItem::ROTATE, fromPose, toPose);
    rotation = new SolutionType(node);
}

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

                ROS_DEBUG_STREAM_NAMED("motion_controller", "Sending VELOCITY: " << currentCommand_);
                pubCmdVel_.publish(messageTwist);
            }
            break;

        case BRAKE:
            // For now the BRAKE is implemented with a 0 velocity. However
            // this can be changed to send a specific command to the brainstem
            currentCommand_ = Velocity<>();

            messageTwist.linear.x = currentCommand_.linear;
            messageTwist.linear.y = 0;
            messageTwist.linear.z = 0;
            messageTwist.angular.x = 0;
            messageTwist.angular.y = 0;
            messageTwist.angular.z = currentCommand_.angular;

            ROS_DEBUG_STREAM_NAMED("motion_controller", "Sending BRAKE");
            pubCmdVel_.publish(messageTwist);
            break;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::executeFinalRotation()
{
    double finalTheta = 0.0;

    if (isScheduledNext(TaskEnum::ROTATE))
    {
        // Remove the scheduled rotation from the work queue
        TaskEnum rotateTask;
        SolutionType rotateSolution;

        popWorkItem(rotateTask, rotateSolution);

        // Extract the final theta from the goal
        finalTheta = rotateSolution.getGoal().toPose.theta;
    }
    else
    {
        finalTheta = activeController_->getGoal().theta;
    }

    // If the current angle of the robot is not what was asked
    if (!AngleMath::equalRad<double>(finalTheta, currentPose_.theta,
        robot_.rotationGoalReachedAngle))
    {
        ROS_DEBUG_STREAM_NAMED("motion_controller", "Rotation needed from: " <<
            AngleMath::rad2deg<double>(currentPose_.theta) << "deg to: " <<
            AngleMath::rad2deg<double>(finalTheta) << "deg");

        SolutionType* solution = GridSolutionFactory::fromRotation(
            currentPose_,
            currentPose_.theta, finalTheta);

        prependWorkItem(TaskEnum::ROTATE, solution);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::checkMotionStatus()
{
    // If the stand controller is currently active and there is work in
    // the queue, cancel work in the controller and pump work from the queue
    if (isStandControllerActive() && isWorkPending())
    {
        ROS_DEBUG_NAMED("MotionController", "Abandoned STAND");
        activeController_->cancel();
    }

    // If the controller says that the goal has been reached, and
    // the work has not been canceled for some reason
    if (activeController_->isGoalReached())
    {
        if (!activeController_->isCanceled())
        {
            ROS_DEBUG_STREAM_NAMED("motion_controller", "Controller " <<
                activeController_->getName() << " reached its goal");

            // If the active controller is the path controller, it is not
            // guaranteed that the final angle of the robot matches the angle
            // of the goal. In that case, a rotation is added to complete the movement
            if (isPathControllerActive())
            {
                // Try to optimize rotations checking the next work scheduled
                // and the current orientation of the robot
                executeFinalRotation();
            }

            // If the robot was using moving controllers, but everything has
            // been completed, declare that the requested goal was reached
            if (isMovingControllerActive() && !isWorkPending())
            {
                setHasArrived(true);
            }
        }
        else
        {
            ROS_DEBUG_STREAM_NAMED("motion_controller", "Controller " <<
                activeController_->getName() << " goal canceled");
        }

        // If no emergency has been declared, consider more work
        if (!isEmergencyDeclared())
        {
            // If there is nothing else to do, simply stand still
            if (!isWorkPending())
            {
                ROS_DEBUG_NAMED("MotionController", "No work was found");
                pushWorkItem(TaskEnum::STAND);
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
void MotionController::popWorkItem(TaskEnum& task, SolutionType& solution)
{
    // First, find the work to do
    WorkType work = work_.front();
    work_.pop_front();

    // Schedule the next task
    task = static_cast<TaskEnum>(work.first);

    // Specify the next solution (if any)
    if (work.second)
    {
        solution = *work.second;
        delete work.second;
    }
    else
    {
        solution.clear();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::prependWorkItem(TaskEnum task, SolutionType* solution)
{
    ROS_DEBUG_STREAM_NAMED("motion_controller", printWorkToString());

    ROS_DEBUG_STREAM_NAMED("motion_controller", "Pushing task: " << TASK_NAMES[task]);
    if (solution)
    {
        ROS_DEBUG_STREAM_NAMED("motion_controller", "Pushing solution: " << *solution);
    }
    else
    {
        ROS_DEBUG_STREAM_NAMED("motion_controller", "Pushing solution: (null)");
    }

    work_.push_back(WorkType(task, solution));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::publishLookAheadDistance()
{
    std_msgs::Float64 messageLookAhead;
    messageLookAhead.data = pathController_->getLookAheadDistance();

    pubLookAheadDistance_.publish(messageLookAhead);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::pumpWorkFromQueue()
{
    // Pop a work item from the queue
    popWorkItem(currentTask_, currentSolution_);

    // Select the appropriate controller for the task
    selectController(currentTask_);

    switch (currentTask_)
    {
        ROS_DEBUG_STREAM_NAMED("motion_controller", "Requested task: " << TASK_NAMES[currentTask_]);

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
void MotionController::pushWorkItem(TaskEnum task, SolutionType* solution)
{
    ROS_DEBUG_STREAM_NAMED("motion_controller", printWorkToString());

    ROS_DEBUG_STREAM_NAMED("motion_controller", "Pushing task: " << TASK_NAMES[task]);
    if (solution)
    {
        ROS_DEBUG_STREAM_NAMED("motion_controller", "Pushing solution: " << *solution);
    }
    else
    {
        ROS_DEBUG_STREAM_NAMED("motion_controller", "Pushing solution: (null)");
    }

    work_.push_back(WorkType(task, solution));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MotionController::pushWorkSolution(SolutionType& solution)
{
    // Create a copy of the specified solution
    SolutionType* pathSolution = new SolutionType(solution);

    SolutionType* initialRotationSolution = nullptr;
    aggregateInitialRotations(currentPose_, pathSolution, initialRotationSolution);

    SolutionType* finalRotationSolution = nullptr;
    aggregateFinalRotations(pathSolution, finalRotationSolution);

    // Push the solutions into the work queue
    if (initialRotationSolution)
    {
        pushWorkItem(TaskEnum::ROTATE, initialRotationSolution);
        currentFinalGoal_ = initialRotationSolution->getGoal().toPose;
    }

    // If the path solution is not empty, push it into
    // the work queue
    if (!pathSolution->empty())
    {
        pushWorkItem(TaskEnum::PATH_FOLLOW, pathSolution);
        currentFinalGoal_ = pathSolution->getGoal().toPose;
    }
    else
    {
        // Otherwise get rid of it
        delete pathSolution;
    }

    if (finalRotationSolution)
    {
        pushWorkItem(TaskEnum::ROTATE, finalRotationSolution);
        currentFinalGoal_ = finalRotationSolution->getGoal().toPose;
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

    ROS_DEBUG_STREAM_NAMED("motion_controller", "Switching to controller " <<
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
    if (!currentSolution_.empty())
    {
        // Store what goal the controller is going to work on
        currentShortTermGoal_ = currentSolution_.getGoal().toPose;

        // Calculate the trajectory
        Trajectory<> trajectory;
        GridTrajectoryGenerator converter(robot_);

        converter.fromSolution(currentSolution_);
        converter.getTrajectory(trajectory);

        ROS_DEBUG_STREAM_NAMED("motion_controller", "Trajectory: " << trajectory);

        // Pass the trajectory to the path controller
        pathController_->setTrajectory(trajectory, currentPose_);
    }
    else
    {
        // Store what goal the controller is going to work on
        currentShortTermGoal_ = currentPose_;

        // If an empty solution was passed, the goal that the
        // Path Controller is going to work on is the current
        // position. In the next cycle, the controller itself will declare
        // that the robot has arrived to the goal
    }
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

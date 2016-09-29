#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>

#include <srslib_framework/Pose.h>
#include <srslib_framework/MsgSolution.h>

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/PoseAdapter.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
#include <srslib_framework/ros/message/SolutionMessageFactory.hpp>
#include <srslib_framework/ros/service/RosCallEmpty.hpp>
#include <srslib_framework/ros/service/RosCallSetBool.hpp>
#include <srslib_framework/ros/service/RosCallSolution.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive(string nodeName) :
    buttonX_(true),
    currentGoal_(),
    currentSolution_(nullptr),
    rosNodeHandle_(nodeName),
    arrived_(true),
    robotInitialPose_(Pose<>::INVALID),
    isJoystickLatched_(false),
    pubExternalArrived_(ChuckTopics::external::RESPONSE_ARRIVED),
    pubGoalToNavigation_(ChuckTopics::internal::GOTO_GOAL),
    pubStatusGoalTarget_(ChuckTopics::internal::GOAL_TARGET_AREA),
    pubInternalInitialPose_(ChuckTopics::internal::INITIAL_POSE, 1),
    pubStatusGoal_(ChuckTopics::internal::GOAL_GOAL, 1)
{
    pubStatusGoalPlan_ = rosNodeHandle_.advertise<nav_msgs::Path>(
        ChuckTopics::internal::GOAL_PATH, 1);

    executeInitialPose();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::run()
{
    connectAllTaps();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        stepChecks();
        stepExecutiveFunctions();

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::connectAllTaps()
{
    tapMap_.connectTap();
    tapOperationalState_.connectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::disconnectAllTaps()
{
    tapMap_.disconnectTap();
    tapOperationalState_.disconnectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executeArrived()
{
    pubExternalArrived_.publish(arrived_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executeInitialPose()
{
    publishInternalInitialPose(robotInitialPose_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePause()
{
    if (RosCallSetBool::call("srsnode_motion", "/trigger/pause", true))
    {
//        if (currentSolution_)
//        {
//            // Reset the solution and publish it
//            // so that every display can update as well. The final
//            // goal remains, so that replan can be executed upon
//            // an resume command
//            currentSolution_->clear();
//            publishInternalGoalSolution(currentSolution_);
//
//            // Remove the current solution as it will have to be calculated
//            // again once the robot is unpaused
//            delete currentSolution_;
//            currentSolution_ = nullptr;
//        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePlanToGoal()
{
    if (!currentRobotPose_.isValid())
    {
        ROS_ERROR_STREAM_NAMED("executive", "Attempting to plan to goal " << currentGoal_ <<
            " from invalid starting pose");
        return;
    }

    if (!currentGoal_.isValid())
    {
        ROS_ERROR_STREAM_NAMED("executive",
            "Attempting to plan to an invalid goal from pose " << currentRobotPose_);
        return;
    }

    Chuck chuck;

    // The requested goal is transformed so that it coincides with
    // where the robot screen will be
    currentTarget_ = PoseMath::translate<double>(currentGoal_, chuck.bodyDepth / 2.0, 0.0);

    // Send the new goal to the navigation system for execution
    pubGoalToNavigation_.publish(currentTarget_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePlanToMove()
{
//    Pose<> newGoal = goal;
//
//    if (abs(PoseMath::measureAngle(newGoal, robotCurrentPose_)) < 0.2)
//    {
//        Pose<> newPose = robotCurrentPose_;
//        newPose.theta = goal.theta;
//
//        double distance = PoseMath::euclidean<double>(goal, robotCurrentPose_);
//        newGoal = PoseMath::transform<double>(newPose, distance);
//    }
//
//    currentGoal_ = newGoal;
//    publishInternalGoal(currentGoal_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executeShutdown()
{
    vector<string> nodes;
    findActiveNodes(nodes);

    for (auto node : nodes)
    {
        RosCallEmpty::call(node, "/trigger/shutdown");
    }

    ros::shutdown();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executeUnpause()
{
    RosCallSetBool::call("srsnode_motion", "/trigger/pause", false);

    if (isExecutingSolution())
    {
        executePlanToGoal();
//        publishInternalGoalSolution(currentSolution_);

        if (currentSolution_)
        {
            RosCallSolution::call("srsnode_motion", "/trigger/execute_solution", currentSolution_);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::findActiveNodes(vector<string>& nodes)
{
    nodes.clear();

    ros::V_string rosMasterNodes;
    ros::master::getNodes(rosMasterNodes);

    string nameSpace = rosNodeHandle_.getNamespace();

    for (auto node : rosMasterNodes)
    {
        if (node.find("srsnode") != string::npos &&
            node.find(nameSpace) == string::npos)
        {
            nodes.push_back(node);
            ROS_INFO_STREAM(node);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishGoalTarget(Pose<> goalTargetArea)
{
    vector<Pose<>> targetArea = PoseMath::pose2Polygon(goalTargetArea, 0.0, 0.0, 0.2, 0.2);
    pubStatusGoalTarget_.publish(targetArea);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishInternalInitialPose(Pose<> initialPose)
{
    if (initialPose.isValid())
    {
        pubInternalInitialPose_.publish(initialPose);
    }
    else
    {
        ROS_ERROR_NAMED("executive", "Trying to publish a invalid initial pose");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::stepChecks()
{
    Pose<> updatedRobotPose = tapInternal_RobotPose_.peek();

    // Run these checks only if the joystick hasn't latched
    if (isExecutingSolution())
    {
        // If the difference between the most updated robot pose and
        // what Executive knew about the robot is greater than the
        // specified threshold, something must have
        // happened with the localization subsystem. Make sure to replan
        if (PoseMath::euclidean(currentRobotPose_, updatedRobotPose) > MAX_RELOCATION_THRESHOLD)
        {
//            taskPlanToGoal();
        }
    }

    currentRobotPose_ = updatedRobotPose;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::stepExecutiveFunctions()
{
    if (tapCmdShutdown_.isNewValueTrue())
    {
        executeShutdown();
    }

    // If there is a new goal to reach
    if (tapCmdMove_.newDataAvailable())
    {
        currentGoal_ = tapCmdMove_.pop();
        executePlanToMove();
    }

    // If there is a new goal to reach
    if (tapCmdGoal_.newDataAvailable())
    {
        // Make sure that the current goal is properly
        // initialized to the closest 90deg angle
        currentGoal_ = tapCmdGoal_.pop();
        currentGoal_.theta = AngleMath::normalizeRad2Rad90(currentGoal_.theta);

        taskPlanToGoal();
    }

    if (tapCmdInitialPose_.newDataAvailable())
    {
        robotInitialPose_ = tapCmdInitialPose_.pop();
        executeInitialPose();
    }

    if (tapInternal_GoalArrived_.newDataAvailable())
    {
        arrived_ = tapInternal_GoalArrived_.pop();
        executeArrived();
    }

    if (tapOperationalState_.newDataAvailable())
    {
        if (tapOperationalState_.isPauseChanged())
        {
            taskPauseChange();
        }
    }

    // If the joystick was touched, check if a custom action was requested
    if (tapJoyAdapter_.newDataAvailable())
    {
        if (tapJoyAdapter_.getButtonAction())
        {
            taskCustomAction();
        }

        // Toggle the X button
        if (tapJoyAdapter_.getButtonX())
        {
            buttonX_ = !buttonX_;
        }

        // Store the latch state for later use
        isJoystickLatched_ = tapJoyAdapter_.getLatchState();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::taskCustomAction()
{
//    currentGoal_ = currentRobotPose_;
//    currentGoal_.theta += M_PI;
//
//    currentSolution_ = GridSolutionFactory::fromRotation(currentRobotPose_,
//        currentRobotPose_.theta,
//        currentRobotPose_.theta + (2 * M_PI - 0.1));
//
////    currentGoal_ = AngleMath::equalRad<double>(currentRobotPose_.theta, 0.0, 1.0) ?
////        Pose<>(15.50, 5.26, 0) :
////        Pose<>(6.38, 5.26, AngleMath::deg2rad<double>(180.0));
//

    currentGoal_ = PoseMath::rotate(currentRobotPose_, M_PI_2);

    // CCW and CW paths for UMBmark paths
    Map* map = tapMap_.getMap();

    // Deallocate the current solution if there is one
    if (currentSolution_)
    {
        delete currentSolution_;
    }

    double deg90 = AngleMath::deg2Rad<double>(90);
    double deg180 = AngleMath::deg2Rad<double>(180);
    double delta = 3;

    vector<Pose<>> goals;
    if (buttonX_)
    {
        // Move the robot CCW
        goals = {
            PoseMath::add(currentRobotPose_, Pose<>(delta, 0, 0)),
            PoseMath::add(currentRobotPose_, Pose<>(delta, delta, deg90)),
            PoseMath::add(currentRobotPose_, Pose<>(0, delta, deg180)),
            PoseMath::add(currentRobotPose_, Pose<>(0, 0, 0))
        };
    }
    else
    {
        // Move the robot CW
        goals = {
            PoseMath::add(currentRobotPose_, Pose<>(0, delta, deg90)),
            PoseMath::add(currentRobotPose_, Pose<>(delta, delta, 0)),
            PoseMath::add(currentRobotPose_, Pose<>(delta, 0, -deg90)),
            PoseMath::add(currentRobotPose_, Pose<>(0, 0, 0))
        };
    }
    currentSolution_ = GridSolutionFactory::fromConsecutiveGoals(map, currentRobotPose_, goals);

    publishGoalTarget(currentGoal_);
//    publishInternalGoalSolution(currentSolution_);

    if (currentSolution_)
    {
        RosCallSolution::call("srsnode_motion", "/trigger/execute_solution", currentSolution_);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::taskPauseChange()
{
    if (tapOperationalState_.getPause())
    {
        executePause();
    }
    else
    {
        executeUnpause();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::taskPlanToGoal()
{
    executePlanToGoal();
    publishGoalTarget(currentGoal_);
//    publishInternalGoalSolution(currentSolution_);

    if (currentSolution_)
    {
        RosCallSolution::call("srsnode_motion", "/trigger/execute_solution", currentSolution_);
    }
}

} // namespace srs

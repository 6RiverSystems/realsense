#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>

#include <srslib_framework/MsgPose.h>
#include <srslib_framework/MsgSolution.h>

#include <srslib_framework/math/PoseMath.hpp>

#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/PoseAdapter.hpp>

#include <srslib_framework/ros/message/SolutionMessageFactory.hpp>
#include <srslib_framework/ros/service/RosServiceCall.hpp>

#include <srslib_framework/robotics/robot/Chuck.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive(string nodeName) :
    currentGoal_(),
    currentSolution_(nullptr),
    rosNodeHandle_(nodeName),
    arrived_(true),
    robotInitialPose_(Pose<>::INVALID)
{
    pubInternalInitialPose_ = rosNodeHandle_.advertise<srslib_framework::MsgPose>(
        "/internal/command/initial_pose", 1);
    pubInternalGoalSolution_ = rosNodeHandle_.advertise<srslib_framework::MsgSolution>(
        "/internal/state/goal/solution", 1);
    pubExternalArrived_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/response/arrived", 1);
    pubStatusGoal_ = rosNodeHandle_.advertise<srslib_framework::MsgPose>(
        "/internal/state/goal/goal", 1);

    pubStatusGoalPlan_ = rosNodeHandle_.advertise<nav_msgs::Path>(
        "/internal/state/goal/path", 1);
    pubStatusGoalTarget_ = rosNodeHandle_.advertise<geometry_msgs::PolygonStamped>(
        "/internal/state/goal/target_area", 1);

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

        currentRobotPose_ = tapInternal_RobotPose_.getPose();

        stepExecutiveFunctions();

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::connectAllTaps()
{
    tapCmdGoal_.connectTap();
    tapCmdInitialPose_.connectTap();
    tapCmdMove_.connectTap();
    tapCmdShutdown_.connectTap();
    tapInternal_GoalArrived_.connectTap();
    tapInternal_RobotPose_.connectTap();

    tapMap_.connectTap();
    tapOperationalState_.connectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::disconnectAllTaps()
{
    tapCmdGoal_.disconnectTap();
    tapCmdInitialPose_.disconnectTap();
    tapCmdMove_.disconnectTap();
    tapCmdShutdown_.disconnectTap();
    tapInternal_GoalArrived_.disconnectTap();
    tapInternal_RobotPose_.disconnectTap();

    tapMap_.disconnectTap();
    tapOperationalState_.disconnectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executeArrived()
{
    std_msgs::Bool messageGoalArrived;
    messageGoalArrived.data = arrived_;

    pubExternalArrived_.publish(messageGoalArrived);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executeInitialPose()
{
    publishInternalInitialPose(robotInitialPose_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePause()
{
    if (RosServiceCall::callSetBool("srsnode_motion", "/trigger/pause", true))
    {
        if (currentSolution_)
        {
            // Reset the solution and publish it
            // so that every display can update as well. The final
            // goal remains, so that replan can be executed upon
            // an resume command
            currentSolution_->clear();
            publishInternalGoalSolution(currentSolution_);

            // Remove the current solution as it will have to be calculated
            // again once the robot is unpaused
            delete currentSolution_;
            currentSolution_ = nullptr;
        }
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
    currentTarget_ = PoseMath::transform<double>(currentGoal_, chuck.bodyDepth / 2);

    Map* map = tapMap_.getMap();
    algorithm_.setGraph(map->getGrid());

    // Prepare the start position for the search
    Grid2d::LocationType internalStart;
    int startAngle;
    PoseAdapter::pose2Map(currentRobotPose_, map, internalStart, startAngle);

    // Prepare the goal position for the search
    Grid2d::LocationType internalGoal;
    int goalAngle;
    PoseAdapter::pose2Map(currentTarget_, map, internalGoal, goalAngle);

    ROS_DEBUG_STREAM_NAMED("executive", "Looking for a path between " <<
        currentRobotPose_ <<
        " (" << internalStart.x << "," << internalStart.y << "," << startAngle << ") and " <<
        currentTarget_ << " - " << currentGoal_ << " " <<
        " (" << internalGoal.x << "," << internalGoal.y << "," << goalAngle << ")");

    bool foundSolution = algorithm_.search(
        SearchPosition<Grid2d>(internalStart, startAngle),
        SearchPosition<Grid2d>(internalGoal, goalAngle));

    if (foundSolution)
    {
        SearchNode<Grid2d>* goalNode = algorithm_.getSolution();

        // Deallocate the current solution if there is one
        if (currentSolution_)
        {
            delete currentSolution_;
        }

        currentSolution_ = GridSolutionFactory::fromSearch(goalNode, map);
        ROS_DEBUG_STREAM_NAMED("executive", "Found solution: " << endl << *currentSolution_);
    }
    else
    {
        ROS_DEBUG_STREAM_NAMED("executive", "Path not found between " <<
            currentRobotPose_ <<
            " (" << internalStart.x << "," << internalStart.y << "," << startAngle << ") and " <<
            currentGoal_ <<
            " (" << internalGoal.x << "," << internalGoal.y << "," << goalAngle << ")");
     }
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
        RosServiceCall::callEmpty(node, "/trigger/shutdown");
    }

    ros::shutdown();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executeUnpause()
{
    RosServiceCall::callSetBool("srsnode_motion", "/trigger/pause", false);

    if (!arrived_)
    {
        executePlanToGoal();
        publishInternalGoalSolution(currentSolution_);
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
    vector<Pose<>> targetArea = PoseMath::pose2polygon(goalTargetArea, 0.0, 0.2, 0.2);

    geometry_msgs::PolygonStamped messageLanding;

    messageLanding.header.frame_id = "map";
    messageLanding.header.stamp = ros::Time::now();

    vector<geometry_msgs::Point32> polygon;

    for (auto pose : targetArea)
    {
        geometry_msgs::Point32 corner;
        corner.x = pose.x;
        corner.y = pose.y;
        corner.z = 0.0;

        polygon.push_back(corner);
    }
    messageLanding.polygon.points = polygon;

    pubStatusGoalTarget_.publish(messageLanding);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishInternalGoalSolution(Solution<GridSolutionItem>* solution)
{
    if (solution)
    {
        ros::Time planningTime = ros::Time::now();

        srslib_framework::MsgSolution messageSolution =
            SolutionMessageFactory::gridSolution2Msg(*solution);
        pubInternalGoalSolution_.publish(messageSolution);

        nav_msgs::Path messagePath = SolutionMessageFactory::gridSolution2PathMsg(*solution);
        pubStatusGoalPlan_.publish(messagePath);
    }
    else
    {
        ROS_ERROR_NAMED("executive", "Trying to publish a null solution");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishInternalInitialPose(Pose<> initialPose)
{
    if (initialPose.isValid())
    {
        srslib_framework::MsgPose message = PoseMessageFactory::pose2Msg(initialPose);
        pubInternalInitialPose_.publish(message);
    }
    else
    {
        ROS_ERROR_NAMED("executive", "Trying to publish a invalid initial pose");
    }
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
        currentGoal_ = tapCmdMove_.getPose();
        executePlanToMove();
    }

    // If there is a new goal to reach
    if (tapCmdGoal_.newDataAvailable())
    {
        currentGoal_ = tapCmdGoal_.getPose();
        executePlanToGoal();
        publishGoalTarget(currentGoal_);
        publishInternalGoalSolution(currentSolution_);
    }

    if (tapCmdInitialPose_.newDataAvailable())
    {
        robotInitialPose_ = tapCmdInitialPose_.getPose();
        executeInitialPose();
    }

    if (tapInternal_GoalArrived_.newDataAvailable())
    {
        arrived_ = tapInternal_GoalArrived_.getBool();
        executeArrived();
    }

    if (tapOperationalState_.newDataAvailable())
    {
        ROS_INFO("State changed");
        if (tapOperationalState_.isPauseChanged() && tapOperationalState_.getPause())
        {
            executePause();
        }

        if (tapOperationalState_.isPauseChanged() && !tapOperationalState_.getPause())
        {
            executeUnpause();
        }
}
}

} // namespace srs

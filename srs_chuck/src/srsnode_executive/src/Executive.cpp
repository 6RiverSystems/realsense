#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <srslib_framework/MsgPose.h>
#include <srslib_framework/MsgSolution.h>

#include <srslib_framework/math/PoseMath.hpp>

#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>

#include <srslib_framework/ros/message/SolutionMessageFactory.hpp>

#include <srslib_framework/robotics/robot/Chuck.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive(string nodeName) :
    currentGoal_(),
    currentSolution_(nullptr),
    rosNodeHandle_(nodeName)
{
    pubInternalInitialPose_ = rosNodeHandle_.advertise<srslib_framework::MsgPose>(
        "/internal/command/initial_pose", 1);
    pubInternalGoalSolution_ = rosNodeHandle_.advertise<srslib_framework::MsgSolution>(
        "/internal/state/goal/solution", 1);
    pubStatusGoalPlan_ = rosNodeHandle_.advertise<nav_msgs::Path>(
        "/internal/state/goal/path", 1);
    pubExternalArrived_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/response/arrived", 1);

    robotInitialPose_ = Pose<>(0, 3.0, 3.0, 0);
    currentRobotPose_ = robotInitialPose_;

    publishInternalInitialPose(robotInitialPose_);
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
    tapCmdPause_.connectTap();
    tapCmdShutdown_.connectTap();
    tapInternal_GoalArrived_.connectTap();
    tapInternal_RobotPose_.connectTap();

    tapMap_.connectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::disconnectAllTaps()
{
    tapCmdGoal_.disconnectTap();
    tapCmdInitialPose_.disconnectTap();
    tapCmdMove_.disconnectTap();
    tapCmdPause_.disconnectTap();
    tapCmdShutdown_.disconnectTap();
    tapInternal_GoalArrived_.disconnectTap();
    tapInternal_RobotPose_.disconnectTap();

    tapMap_.disconnectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executeArrived()
{
    std_msgs::Bool messageGoalArrived;
    messageGoalArrived.data = tapInternal_GoalArrived_.getBool();

    pubExternalArrived_.publish(messageGoalArrived);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executeInitialPose(Pose<> initialPose)
{
    robotInitialPose_ = initialPose;
    publishInternalInitialPose(robotInitialPose_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePause()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePlanToGoal(Pose<> goalPose)
{
    Chuck chuck;

    // The requested goal is transformed so that it coincides with
    // where the robot screen will be
    currentGoal_ = PoseMath::transform<double>(goalPose, chuck.bodyDepth / 2);

    Map* map = tapMap_.getMap();
    algorithm_.setGraph(map->getGrid());

    // Prepare the start position for the search
    int fromR = 0;
    int fromC = 0;
    map->getMapCoordinates(currentRobotPose_.x, currentRobotPose_.y, fromC, fromR);
    Grid2d::LocationType internalStart(fromC, fromR);
    int startAngle = AngleMath::normalizeRad2deg90(currentRobotPose_.theta);

    // Prepare the goal position for the search
    int toR = 0;
    int toC = 0;
    map->getMapCoordinates(currentGoal_.x, currentGoal_.y, toC, toR);
    Grid2d::LocationType internalGoal(toC, toR);
    int goalAngle = AngleMath::normalizeRad2deg90(currentGoal_.theta);

    ROS_DEBUG_STREAM_NAMED("Motion", "Looking for a path between " << currentRobotPose_ << " (" <<
        fromC << "," << fromR << "," << startAngle <<
        ") and " << goalPose << " (" << toC << "," << toR << "," << goalAngle << ")");

    bool foundSolution = algorithm_.search(
        SearchPosition<Grid2d>(internalStart, startAngle),
        SearchPosition<Grid2d>(internalGoal, goalAngle));

    if (foundSolution)
    {
        SearchNode<Grid2d>* goalNode = algorithm_.getSolution();

        currentSolution_ = GridSolutionFactory::fromSearch(goalNode, map);
        ROS_DEBUG_STREAM_NAMED("Executive", "Found solution: " << endl << currentSolution_);
    }
    else
    {
        ROS_ERROR_STREAM_NAMED("Motion", "Path not found between " <<
            currentRobotPose_ << " (" << fromC << "," << fromR << "," << startAngle << ") and " <<
            goalPose << " (" << toC << "," << toR << "," << goalAngle << ")");
     }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePlanToMove(Pose<> goal)
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
        string fullServiceName = node + "/trigger/shutdown";

        ros::ServiceClient client = rosNodeHandle_.serviceClient<std_srvs::Empty>(fullServiceName);
        std_srvs::Empty::Request req;
        std_srvs::Empty::Response resp;

        if (client.call(req, resp))
        {
            ROS_INFO_STREAM("The node " << node << " responded to a shutdown request.");
        }
        else
        {
            ROS_ERROR_STREAM("The node " << node << " did not responded to a shutdown request.");
        }
    }

    ros::shutdown();
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishInternalGoalSolution(Solution<GridSolutionItem>* solution)
{
    ros::Time planningTime = ros::Time::now();

    srslib_framework::MsgSolution messageSolution =
        SolutionMessageFactory::gridSolution2Msg(*solution);
    pubInternalGoalSolution_.publish(messageSolution);

    nav_msgs::Path messagePath = SolutionMessageFactory::gridSolution2PathMsg(*solution);
    pubStatusGoalPlan_.publish(messagePath);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishInternalInitialPose(Pose<> initialPose)
{
    MsgPose message = PoseMessageFactory::pose2Msg(initialPose);
    pubInternalInitialPose_.publish(message);
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
        executePlanToMove(tapCmdMove_.getPose());
    }

    // If there is a new goal to reach
    if (tapCmdGoal_.newDataAvailable())
    {
        executePlanToGoal(tapCmdGoal_.getPose());
        publishInternalGoalSolution(currentSolution_);
    }

    if (tapCmdInitialPose_.newDataAvailable())
    {
        executeInitialPose(tapCmdInitialPose_.getPose());
    }

    if (tapCmdPause_.isNewValueTrue())
    {
        executePause();
    }

    if (tapInternal_GoalArrived_.newDataAvailable())
    {
        executeArrived();
    }
}

} // namespace srs

#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <srslib_framework/MapCoordinates.h>
#include <srslib_framework/MsgPose.h>
using namespace srslib_framework;

#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive(string nodeName) :
    currentGoal_(),
    rosNodeHandle_(nodeName)
{
    pubInternalInitialPose_ = rosNodeHandle_.advertise<MsgPose>("/internal/command/initial_pose", 1);
    pubInternalGoal_ = rosNodeHandle_.advertise<MsgPose>("/internal/command/goal", 1);

    pubExternalArrived_ = rosNodeHandle_.advertise<std_msgs::Bool>("/response/arrived", 1);

    robotInitialPose_ = Pose<>(0, 3.0, 3.0, 0);
    robotCurrentPose_ = robotInitialPose_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::run()
{
    connectAllTaps();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

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
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePause()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePlanToGoal(Pose<> goal)
{
    Chuck chuck;

    // The requested goal is transformed so that it coincides with
    // where the robot screen will be
    currentGoal_ = PoseMath::transform<double>(goal, chuck.bodyDepth / 2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePlanToMove(Pose<> goal)
{
//    Chuck chuck;
//    double angle = PoseMath::measureAngle(goal, robotCurrentPose_);
//
//    // Keep the goal in line with current robot pose
//    goalPose.x = (abs(goalPose.x - robotPose.x) < chuck.bodyWidth / 2) ? robotPose.x : goalPose.x;
//    goalPose.y = (abs(goalPose.y - robotPose.y) < chuck.bodyDepth / 2) ? robotPose.y : goalPose.y;
//
//
    currentGoal_ = goal;
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

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishInternalInitialPose(Pose<> initialPose)
{
    MsgPose message;

    message.header.stamp = ros::Time::now();
    message.x = initialPose.x;
    message.y = initialPose.y;
    message.theta = AngleMath::rad2deg<double>(initialPose.theta);

    pubInternalInitialPose_.publish(message);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishInternalGoal(Pose<> goal)
{
    MsgPose message;

    message.header.stamp = ros::Time::now();
    message.x = goal.x;
    message.y = goal.y;
    message.theta = AngleMath::rad2deg<double>(goal.theta);

    pubInternalGoal_.publish(message);
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
        publishInternalGoal(currentGoal_);
    }

    // If there is a new goal to reach
    if (tapCmdGoal_.newDataAvailable())
    {
        executePlanToGoal(tapCmdGoal_.getPose());
        publishInternalGoal(currentGoal_);
    }

    if (tapCmdInitialPose_.newDataAvailable())
    {
        executeInitialPose(tapCmdInitialPose_.getPose());
        publishInternalInitialPose(robotInitialPose_);
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

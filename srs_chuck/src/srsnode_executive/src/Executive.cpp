#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

#include <srslib_framework/MapCoordinates.h>
using namespace srslib_framework;

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive(string nodeName) :
    currentGoal_(),
    rosNodeHandle_(nodeName)
{
    pubInternalInitialPose_ = rosNodeHandle_.advertise<geometry_msgs::PoseStamped>(
        "/internal/command/initial_pose", 1);
    pubInternalGoal_ = rosNodeHandle_.advertise<geometry_msgs::PoseStamped>(
        "/internal/command/goal", 1);

    pubExternalArrived_ = rosNodeHandle_.advertise<std_msgs::Bool>(
        "/response/arrived", 1);

    robotInitialPose_ = Pose<>(0, 3.0, 3.0, 0);
    robotCurrentPose_ = robotInitialPose_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::run()
{
    connectAllTaps();

    publishInternalInitialPose(robotInitialPose_);

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
    geometry_msgs::PoseStamped message;

    tf::Quaternion quaternion = tf::createQuaternionFromYaw(initialPose.theta);

    message.header.stamp = ros::Time::now();
    message.pose.position.x = initialPose.x;
    message.pose.position.y = initialPose.y;
    message.pose.orientation.x = quaternion.x();
    message.pose.orientation.y = quaternion.y();
    message.pose.orientation.z = quaternion.z();
    message.pose.orientation.w = quaternion.w();

    pubInternalInitialPose_.publish(message);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishInternalGoal(Pose<> goal)
{
    geometry_msgs::PoseStamped message;

    tf::Quaternion quaternion = tf::createQuaternionFromYaw(goal.theta);

    message.header.stamp = ros::Time::now();
    message.pose.position.x = goal.x;
    message.pose.position.y = goal.y;
    message.pose.orientation.x = quaternion.x();
    message.pose.orientation.y = quaternion.y();
    message.pose.orientation.z = quaternion.z();
    message.pose.orientation.w = quaternion.w();

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

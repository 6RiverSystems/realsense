#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <srslib_framework/MapCoordinates.h>
using namespace srslib_framework;

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive(string nodeName) :
    currentGoal_(),
    //inc_(0),
    rosNodeHandle_(nodeName),
    robotCurrentPose_(),
    robotInitialPose_(),
    tapCmdGoal_(rosNodeHandle_),
    tapCmdInitialPose_(rosNodeHandle_),
    tapCmdPause_(rosNodeHandle_),
    tapCmdShutdown_(rosNodeHandle_),
    tapMap_(rosNodeHandle_)
{
    pubInitialPose_ = rosNodeHandle_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "initial_pose", 1);

    robotInitialPose_ = Pose<>(0, 3.0, 2.0, 0);
    robotCurrentPose_ = robotInitialPose_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::run()
{
    connectAllTaps();

    publishInitialPose();

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

    tapMap_.connectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::disconnectAllTaps()
{
    tapCmdGoal_.disconnectTap();
    tapCmdInitialPose_.disconnectTap();
    tapCmdPause_.disconnectTap();
    tapCmdShutdown_.disconnectTap();

    tapMap_.disconnectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executeInitialPose(Pose<> initialPose)
{
    robotInitialPose_ = initialPose;
    //inc_ = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePause()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::executePlanToGoal(Pose<> goal)
{
    currentGoal_ = goal;

//    algorithm_.setGraph(tapMap_.getMap()->getGrid());
//
//    int r = 0;
//    int c = 0;
//
//    tapMap_.getMap()->getMapCoordinates(robotCurrentPose_.x, robotCurrentPose_.y, c, r);
//    Grid2d::LocationType internalStart(c, r);
//
//    // tapMap_.getMap()->getMapCoordinates(goal.x, goal.y, c, r);
//    Grid2d::LocationType internalGoal(c + 10, r);
//
//    algorithm_.search(
//        SearchPosition<Grid2d>(internalStart, 0),
//        SearchPosition<Grid2d>(internalGoal, 0));
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

    cout << nameSpace << endl;

    for (auto node : rosMasterNodes)
    {
        cout << node << endl;
        if (node.find("srsnode") != string::npos &&
            node.find(nameSpace) == string::npos)
        {
            nodes.push_back(node);
            ROS_INFO_STREAM(node);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishInitialPose()
{
    geometry_msgs::PoseWithCovarianceStamped message;

    tf::Quaternion quaternion = tf::createQuaternionFromYaw(robotInitialPose_.theta);

    message.header.stamp = ros::Time::now();
    message.pose.pose.position.x = robotInitialPose_.x;
    message.pose.pose.position.y = robotInitialPose_.y;
    message.pose.pose.orientation.x = quaternion.x();
    message.pose.pose.orientation.y = quaternion.y();
    message.pose.pose.orientation.z = quaternion.z();
    message.pose.pose.orientation.w = quaternion.w();

    pubInitialPose_.publish(message);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::stepExecutiveFunctions()
{
    if (tapCmdShutdown_.isShutdownRequested())
    {
        executeShutdown();
    }

    // If there is a new goal to reach
    if (tapCmdGoal_.newDataAvailable())
    {
        executePlanToGoal(tapCmdGoal_.getGoal());
    }

    if (tapCmdInitialPose_.newDataAvailable())
    {
        executeInitialPose(tapCmdInitialPose_.getRobotPose());
        publishInitialPose();
    }

    if (tapCmdPause_.isPauseRequested())
    {
        executePause();
    }
}

} // namespace srs

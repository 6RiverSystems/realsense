#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <srslib_framework/math/Math.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive(string nodeName) :
    algorithm_(grid_),
    grid_(GRID_SIZE),
    inc_(0),
    rosNodeHandle_("srsnode_executive"),
    robotPose_(),
    robotPose0_(),
    tapGoal_(nodeName),
    tapYouAreHere_(nodeName)
{
    pubPlan_ = rosNodeHandle_.advertise<nav_msgs::Path>("plan", 1);
    pubInitialPose_ = rosNodeHandle_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "initial_pose", 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::run()
{
    tapGoal_.connectTap();
    tapYouAreHere_.connectTap();

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
void Executive::disconnectAllTaps()
{
    tapGoal_.disconnectTap();
    tapYouAreHere_.disconnectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::planToGoal(Pose<> goal)
{
    Grid2d::LocationType start(0, 0);
    Grid2d::LocationType internalGoal(11, 0);

    algorithm_.search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(internalGoal, 0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishPlan()
{
    inc_++;

    vector<SolutionNode<Grid2d>> path = algorithm_.getPath();

    ros::Time planningTime = ros::Time::now();

    cout << "=================== " << planningTime << endl;
    for (auto node : path) {
        cout << node << endl;
    }
    cout << endl;

    nav_msgs::Path message;
    message.header.frame_id = "map";
    message.header.stamp = planningTime;

    std::vector<geometry_msgs::PoseStamped> plan;

    for (auto node : path)
    {
        geometry_msgs::PoseStamped poseStamped;
        tf::Quaternion quaternion = tf::createQuaternionFromYaw(0);

        poseStamped.pose.position.x = 2.0 + node.action.position.location.x;
        poseStamped.pose.position.y = inc_ + node.action.position.location.y;
        poseStamped.pose.orientation.x = quaternion.x();
        poseStamped.pose.orientation.y = quaternion.y();
        poseStamped.pose.orientation.z = quaternion.z();
        poseStamped.pose.orientation.w = quaternion.w();

        plan.push_back(poseStamped);
    }

    message.poses = plan;

    pubPlan_.publish(message);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishPose0()
{
    geometry_msgs::PoseWithCovarianceStamped message;

    tf::Quaternion orientation = tf::createQuaternionFromYaw(robotPose0_.theta);

    message.header.stamp = ros::Time::now();
    message.pose.pose.position.x = robotPose0_.x;
    message.pose.pose.position.y = robotPose0_.y;
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();

    pubInitialPose_.publish(message);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::stepExecutiveFunctions()
{
    // If there is a new goal to reach
    if (tapGoal_.newDataAvailable())
    {
        planToGoal(tapGoal_.getCurrentGoal());
        publishPlan();
    }

    if (tapYouAreHere_.newDataAvailable())
    {
        robotPose0_ = tapYouAreHere_.getRobotPose();
        publishPose0();
    }
}

} // namespace srs

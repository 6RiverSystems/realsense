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
    currentGoal_(),
    grid_(GRID_SIZE),
    inc_(0),
    rosNodeHandle_("srsnode_executive"),
    robotCurrentPose_(),
    robotInitialPose_(),
    tapGoal_(nodeName),
    tapYouAreHere_(nodeName)
{
    pubGoalPlan_ = rosNodeHandle_.advertise<nav_msgs::Path>("state/current_goal/plan", 1);
    pubGoalGoal_ = rosNodeHandle_.advertise<geometry_msgs::PoseStamped>("state/current_goal/goal", 1);
    pubInitialPose_ = rosNodeHandle_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        "state/initial_pose", 1);

    algorithm_.setGraph(&grid_);

    robotInitialPose_ = Pose<>(0, 2.0, 2.0, 0);
    robotCurrentPose_ = robotInitialPose_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::run()
{
    tapGoal_.connectTap();
    tapYouAreHere_.connectTap();

    publishGoal();
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
void Executive::disconnectAllTaps()
{
    tapGoal_.disconnectTap();
    tapYouAreHere_.disconnectTap();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::planToGoal(Pose<> goal)
{
    Grid2d::LocationType start(0, 0);
    Grid2d::LocationType internalGoal(3, 0);

    algorithm_.search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(internalGoal, 0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishGoal()
{
    inc_++;

    vector<SolutionNode<Grid2d>> path = algorithm_.getPath();

    ros::Time planningTime = ros::Time::now();

    cout << "=================== " << planningTime << endl;
    for (auto node : path) {
        cout << node << endl;
    }
    cout << endl;

    nav_msgs::Path messageGoalPlan;
    messageGoalPlan.header.frame_id = "map";
    messageGoalPlan.header.stamp = planningTime;

    vector<geometry_msgs::PoseStamped> planPoses;

    for (auto node : path)
    {
        geometry_msgs::PoseStamped poseStamped;
        tf::Quaternion quaternion = tf::createQuaternionFromYaw(node.action.position.orientation);

        poseStamped.pose.position.x = 2.0 + node.action.position.location.x;
        poseStamped.pose.position.y = inc_ + node.action.position.location.y;
        poseStamped.pose.position.z = 0.0;
        poseStamped.pose.orientation.x = quaternion.x();
        poseStamped.pose.orientation.y = quaternion.y();
        poseStamped.pose.orientation.z = quaternion.z();
        poseStamped.pose.orientation.w = quaternion.w();

        planPoses.push_back(poseStamped);
    }

    messageGoalPlan.poses = planPoses;

    geometry_msgs::PoseStamped messageGoal;
    tf::Quaternion quaternion = tf::createQuaternionFromYaw(currentGoal_.theta);

    messageGoal.header.stamp = planningTime;
    messageGoal.pose.position.x = currentGoal_.x;
    messageGoal.pose.position.y = currentGoal_.y;
    messageGoal.pose.position.z = 0.0;
    messageGoal.pose.orientation.x = quaternion.x();
    messageGoal.pose.orientation.y = quaternion.y();
    messageGoal.pose.orientation.z = quaternion.z();
    messageGoal.pose.orientation.w = quaternion.w();

    pubGoalGoal_.publish(messageGoal);
    pubGoalPlan_.publish(messageGoalPlan);
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
    // If there is a new goal to reach
    if (tapGoal_.newDataAvailable())
    {
        currentGoal_ = tapGoal_.getCurrentGoal();
        planToGoal(currentGoal_);
        publishGoal();
    }

    if (tapYouAreHere_.newDataAvailable())
    {
        robotInitialPose_ = tapYouAreHere_.getRobotPose();
        publishInitialPose();
        inc_ = 0;
    }
}

} // namespace srs

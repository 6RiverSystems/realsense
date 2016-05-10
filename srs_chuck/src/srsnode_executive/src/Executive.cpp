#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <srslib_framework/math/Math.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive() :
    rosNodeHandle_("srsnode_executive"),
    grid_(GRID_SIZE),
    algorithm_(grid_),
    inc_(0)
{
    tapGoal_.connectTap();
    pubPlan_ = rosNodeHandle_.advertise<nav_msgs::Path>("/plan", 1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::run()
{
    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        // If there is a new goal to reach
        if (tapGoal_.newDataAvailable())
        {
            planToGoal(tapGoal_.getCurrentGoal());
        }

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::planToGoal(Pose<> goal)
{
    Grid2d::LocationType start(0, 0);
    Grid2d::LocationType internalGoal(11, 0);

    inc_++;
    algorithm_.search(SearchPosition<Grid2d>(start, 0), SearchPosition<Grid2d>(internalGoal, 0));

    publishPlan();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::publishPlan()
{
    vector<SolutionNode<Grid2d>> path = algorithm_.getPath();

    ros::Time planningTime = ros::Time::now();

    cout << "=================== " << planningTime << endl;
    for (auto node : path) {
        cout << node << endl;
    }
    cout << endl;

    nav_msgs::Path messagePath;
    messagePath.header.frame_id = "map";
    messagePath.header.stamp = planningTime;

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

    messagePath.poses = plan;

    pubPlan_.publish(messagePath);
}

} // namespace srs

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPGOALPLAN_HPP_
#define ROSTAPGOALPLAN_HPP_

#include <string>
#include <vector>
using namespace std;

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

#include <srslib_framework/ros/RosTap.hpp>
#include <srslib_framework/search/SolutionNode.hpp>
#include <srslib_framework/math/Time.hpp>
#include <srslib_framework/graph/grid2d/Grid2d.hpp>

namespace srs {

class RosTapGoalPlan :
    public RosTap
{
public:
    RosTapGoalPlan(ros::NodeHandle rosHandle) :
        RosTap(rosHandle, "Initial Pose Tap")
    {}

    ~RosTapGoalPlan()
    {
        disconnectTap();
    }

    vector<SolutionNode<Grid2d>> getPlan()
    {
        setNewData(false);
        return solution_;
    }

    void reset()
    {
        solution_.clear();
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/srsnode_executive/current_goal/plan", 1,
            &RosTapGoalPlan::onGoalPlan, this);

        return true;
    }

private:
    void onGoalPlan(nav_msgs::PathConstPtr message)
    {
        solution_.clear();

        for (auto node : message->poses)
        {
            SearchAction<Grid2d> action(SearchAction<Grid2d>::FORWARD);
            geometry_msgs::Quaternion quaternion =  node.pose.orientation;

            action.position.location = SearchPosition<Grid2d>::LocationType(
                node.pose.position.x,
                node.pose.position.y);
            action.position.orientation = tf::getYaw(quaternion);

            solution_.push_back(SolutionNode<Grid2d>(action));
        }

        auto goal = solution_.end() - 1;

        SearchAction<Grid2d> action(SearchAction<Grid2d>::GOAL);
        action.position.location = SearchPosition<Grid2d>::LocationType(
            goal->action.position.location.x,
            goal->action.position.location.y);
        action.position.orientation = goal->action.position.orientation;

        solution_.push_back(SolutionNode<Grid2d>(action));

        setNewData(true);
    }

    vector<SolutionNode<Grid2d>> solution_;
};

} // namespace srs

#endif // ROSTAPGOALPLAN_HPP_

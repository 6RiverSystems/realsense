/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <dynamic_reconfigure/server.h>

#include <srsnode_navigation/SrsPlannerConfig.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/ros/channel/ChannelRosPath.hpp>
#include <srslib_framework/ros/tap/TapMapStack.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>

namespace srs {

class SrsPlannerConventional :
    public nav_core::BaseGlobalPlanner,
    public Observer<Subscriber<srslib_framework::MapStack>>
{
public:
    SrsPlannerConventional();
    SrsPlannerConventional(string name, costmap_2d::Costmap2DROS* rosCostMap);
    virtual ~SrsPlannerConventional();

    void initialize(string name, costmap_2d::Costmap2DROS* rosCostMap);

    bool makePlan(const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        vector<geometry_msgs::PoseStamped>& plan);

private:
    void notified(Subscriber<srslib_framework::MapStack>* subject);

    void onConfigChange(srsnode_navigation::SrsPlannerConfig& config, uint32_t level);

    void populatePath(const geometry_msgs::PoseStamped& start,
        const Trajectory<>& trajectory,
        const geometry_msgs::PoseStamped& goal,
        vector<geometry_msgs::PoseStamped>& plan);

    AStar::ConfigParameters astarConfigParameters_;

    ChannelRosPath channelRosPath_;
    dynamic_reconfigure::Server<srsnode_navigation::SrsPlannerConfig>* configServer_;
    costmap_2d::Costmap2D* costMap2d_;

    MapStackNode::SearchParameters nodeSearchParameters_;

    MapStack* srsMapStack_;

    TapMapStack tapMapStack_;
};

} // namespace srs

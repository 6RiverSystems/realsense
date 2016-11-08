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

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/ros/tap/TapMapStack.hpp>

namespace srs {

class SrsPlannerConventional : public nav_core::BaseGlobalPlanner
{
public:
    SrsPlannerConventional();
    SrsPlannerConventional(string name, costmap_2d::Costmap2DROS* rosCostMap);
    virtual ~SrsPlanner();

    void initialize(string name, costmap_2d::Costmap2DROS* rosCostMap);

    bool makePlan(
        const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        vector<geometry_msgs::PoseStamped>& plan);

private:
    void initializeParams();

    void updateMapStack(costmap_2d::Costmap2DROS* rosCostMap);

    MapStack* srsMapStack_;

    TapMapStack tapMapStack_;
};

} // namespace srs

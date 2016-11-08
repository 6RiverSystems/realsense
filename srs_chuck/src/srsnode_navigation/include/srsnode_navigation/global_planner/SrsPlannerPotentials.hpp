/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <vector>
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
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/ros/tap/TapMapStack.hpp>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/GetPlan.h>
#include <dynamic_reconfigure/server.h>

#include <srsnode_navigation/global_planner/AStarPotentials.hpp>
#include <srsnode_navigation/global_planner/OrientationFilter.hpp>
using namespace srs;

namespace srs {

class SrsPlannerPotentials : public nav_core::BaseGlobalPlanner
{
public:
    SrsPlannerPotentials();
    SrsPlannerPotentials(string name, costmap_2d::Costmap2DROS* rosCostMap);
    virtual ~SrsPlannerPotentials();

    void initialize(string name, costmap_2d::Costmap2DROS* rosCostMap);
    void initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);

    bool makePlan(
        const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        vector<geometry_msgs::PoseStamped>& plan);
    bool makePlan(
        const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        double tolerance,
        std::vector<geometry_msgs::PoseStamped>& plan);

private:
    void getPlanFromPotential(std::vector<std::pair<float, float>>& path,
        std::vector<geometry_msgs::PoseStamped>& plan);

    void initializeParams();

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    void publishPotential(float* potential);

    void updateMapStack(costmap_2d::Costmap2D* rosCostMap);

    bool allow_unknown_;
    AStarPotentials* astar_;

    costmap_2d::Costmap2D* costmap_;

    double default_tolerance_;

    std::string frame_id_;

    bool initialized_;

    boost::mutex mutex_;

    OrientationFilter* orientation_filter_;

    ros::Publisher plan_pub_;
    double planner_window_x_;
    double planner_window_y_;
    ros::Publisher potential_pub_;
    float* potential_array_;
    bool publish_potential_;
    int publish_scale_;

//    unsigned int start_x_;
//    unsigned int start_y_;
//    unsigned int end_x_;
//    unsigned int end_y_;

    MapStack* srsMapStack_;

    TapMapStack tapMapStack_;
    std::string tf_prefix_;

    bool visualize_potential_;
};

} // namespace srs

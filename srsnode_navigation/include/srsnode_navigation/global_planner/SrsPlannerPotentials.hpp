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

#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>
#include <srslib_framework/ros/tap/TapMapStack.hpp>

#include <srsnode_navigation/SrsPlannerConfig.h>

#include <srsnode_navigation/global_planner/potentials/AStarPotentials.hpp>
#include <srsnode_navigation/global_planner/potentials/OrientationFilter.hpp>
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
    void extractQueuePolygons();

    void getPlanFromPotential(std::vector<std::pair<float, float>>& path,
        std::vector<geometry_msgs::PoseStamped>& plan);

    void onConfigChange(srsnode_navigation::SrsPlannerConfig& config, uint32_t level);

    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    void publishPotential(float* potential);

    void updateMapStack(costmap_2d::Costmap2D* rosCostMap);

    bool allowUnknown_;
    AStarPotentials* astar_;

    dynamic_reconfigure::Server<srsnode_navigation::SrsPlannerConfig>* configServer_;
    costmap_2d::Costmap2D* costMap_;

    bool initialized_;

    unsigned int lethalCost_;
    unsigned int logicalCostRatio_;

    boost::mutex mutex_;

    unsigned int neutralCost_;

    OrientationFilter* orientationFilter_;

    ros::Publisher planPublisher_;
    ros::Publisher potentialPublisher_;
    float* potentialArray_;
    bool publishPotential_;
    int publishScale_;

    AStarPotentials::QueueMapType queuesMap_;

    MapStack* srsMapStack_;

    TapMapStack tapMapStack_;
    std::string tfFrameid_;
    std::string tfPrefix_;

    bool useGridPath_;
    bool useQuadratic_;

    unsigned int weightRatio_;
};

} // namespace srs

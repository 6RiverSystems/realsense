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

    bool computePotential(const geometry_msgs::Point& world_point);

    bool getPlanFromPotential(double start_x, double start_y, double end_x, double end_y,
                              const geometry_msgs::PoseStamped& goal,
                              std::vector<geometry_msgs::PoseStamped>& plan);
    double getPointPotential(const geometry_msgs::Point& world_point);

    bool validPointPotential(const geometry_msgs::Point& world_point);
    bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

protected:

    costmap_2d::Costmap2D* costmap_;
    std::string frame_id_;
    ros::Publisher plan_pub_;
    bool initialized_, allow_unknown_, visualize_potential_;

private:
    void initializeParams();

    void updateMapStack(costmap_2d::Costmap2D* rosCostMap);

//    void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
    void publishPotential(float* potential);

    void getPlanFromPotential(std::vector<std::pair<float, float>>& path,
//        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan);

    double planner_window_x_;
    double planner_window_y_;
    double default_tolerance_;

    std::string tf_prefix_;
    boost::mutex mutex_;
//    ros::ServiceServer make_plan_srv_;

//    PotentialCalculator* p_calc_;
//    Expander* planner_;
//    Traceback* path_maker_;
    OrientationFilter* orientation_filter_;

    bool publish_potential_;
    ros::Publisher potential_pub_;
    int publish_scale_;

//    void outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value);
//    unsigned char* cost_array_;
    float* potential_array_;
    unsigned int start_x_, start_y_, end_x_, end_y_;

    AStarPotentials* astar_;

//    bool old_navfn_behavior_;

//    dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig> *dsrv_;
//    void reconfigureCB(global_planner::GlobalPlannerConfig &config, uint32_t level);

    MapStack* srsMapStack_;

    TapMapStack tapMapStack_;
};

} // namespace srs

#include <srsnode_navigation/global_planner/SrsPlannerPotentials.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(srs::SrsPlannerPotentials, nav_core::BaseGlobalPlanner)

#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <srsnode_navigation/global_planner/QuadraticCalculator.hpp>
#include <srsnode_navigation/global_planner/GradientPath.hpp>
#include <srsnode_navigation/global_planner/AStarExpansion.hpp>

#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlannerPotentials::SrsPlannerPotentials() :
    srsMapStack_(nullptr),
    costmap_(NULL),
    initialized_(false),
    allow_unknown_(true),
    astar_(nullptr)
{
    ROS_WARN("SrsPlannerPotentials::SrsPlannerPotentials() called");

    initializeParams();
    updateMapStack(nullptr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlannerPotentials::SrsPlannerPotentials(string name, costmap_2d::Costmap2DROS* rosCostMap) :
    srsMapStack_(nullptr),
    costmap_(NULL),
    initialized_(false),
    allow_unknown_(true),
    astar_(nullptr)
{
    ROS_WARN("SrsPlannerPotentials::SrsPlannerPotentials(...) called");

    initialize(name, rosCostMap);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlannerPotentials::~SrsPlannerPotentials()
{
    delete srsMapStack_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::initialize(std::string name, costmap_2d::Costmap2D* costmap,
    std::string frame_id)
{
    if (!initialized_)
    {
        ROS_WARN("SrsPlannerPotentials::initialize(...) called");

        initializeParams();
        //updateMapStack(costmap);

        costmap_ = costmap;
        frame_id_ = frame_id;

        orientation_filter_ = new OrientationFilter();

        ros::NodeHandle private_nh("~/" + name);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);

        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        potential_array_ = nullptr;
        initialized_ = true;
    }
    else
    {
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::initialize(std::string name, costmap_2d::Costmap2DROS* rosCostMap)
{
    ROS_WARN("SrsPlanner::initialize() called");

    initialize(name, rosCostMap->getCostmap(), rosCostMap->getGlobalFrameID());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool SrsPlannerPotentials::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    vector<geometry_msgs::PoseStamped>& plan)
{
    ROS_WARN("SrsPlanner::makePlan() called");

    // Make sure that we get the latest Map Stack
    updateMapStack(nullptr);

    return makePlan(start, goal, default_tolerance_, plan);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool SrsPlannerPotentials::makePlan(
    const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
    double tolerance, std::vector<geometry_msgs::PoseStamped>& path)
{
    boost::mutex::scoped_lock lock(mutex_);

    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }

    path.clear();

    ros::NodeHandle n;
    std::string global_frame = frame_id_;

    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame))
    {
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
            tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame))
    {
        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
            tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }

    astar_ = new AStarPotentials(srsMapStack_->getLogicalMap(), costmap_);

    // clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);

    std::vector<std::pair<float, float>> plan;

    bool found = astar_->calculatePath(
        start.pose.position.x, start.pose.position.y,
        goal.pose.position.x, goal.pose.position.y,
        plan,
        potential_array_);

    publishPotential(potential_array_);

    if (found)
    {
        getPlanFromPotential(plan, path);

        geometry_msgs::PoseStamped goal_copy = goal;
        goal_copy.header.stamp = ros::Time::now();
        path.push_back(goal_copy);

        // add orientations if needed
        orientation_filter_->processPath(start, path);

        publishPlan(path);
    }
    else
    {
        ROS_ERROR("Failed to get a plan. Saving map.");
        costmap_->saveMap("cost_map.pgm");
    }


    delete potential_array_;
    potential_array_ = nullptr;

    delete astar_;
    astar_ = nullptr;

    return !path.empty();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::getPlanFromPotential(
    std::vector<std::pair<float, float>>& path,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
    plan.clear();

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() - 1; i>=0; i--)
    {
        std::pair<float, float> point = path[i];

        double world_x;
        double world_y;
        astar_->mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = frame_id_;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::initializeParams()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty())
    {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::publishPotential(float* potential)
{
    int nx = costmap_->getSizeInCellsX();
    int ny = costmap_->getSizeInCellsY();

    double resolution = costmap_->getResolution();

    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = frame_id_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;

    grid.info.width = nx;
    grid.info.height = ny;

    double wx;
    double wy;
    costmap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        float potential = potential_array_[i];
        if (potential < POT_HIGH)
        {
            if (potential > max)
            {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        if (potential_array_[i] >= POT_HIGH)
        {
            grid.data[i] = -1;
        }
        else
        {
            grid.data[i] = potential_array_[i] * publish_scale_ / max;
        }
    }

    potential_pub_.publish(grid);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::updateMapStack(costmap_2d::Costmap2D* rosCostMap)
{
    // Make sure that the neither the logical not the occupancy maps
    // have been re-published. In case, destroy what we have and
    // ask for a new stack
    if (tapMapStack_.newDataAvailable())
    {
        delete srsMapStack_;
        srsMapStack_ = tapMapStack_.pop();
    }

    // TODO: Sync the obstruction map in the stack with the new rosCostMap
}

} // namespace srs


#include <srsnode_navigation/global_planner/SrsPlannerPotentials.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(srs::SrsPlannerPotentials, nav_core::BaseGlobalPlanner)

#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

//#include <global_planner/dijkstra.h>
//#include <global_planner/astar.h>
//#include <global_planner/grid_path.h>
//#include <global_planner/gradient_path.h>
//#include <global_planner/quadratic_calculator.h>

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
    logicalMap_(nullptr)
{
    ROS_WARN("SrsPlanner::SrsPlanner() called");

    initializeParams();
    updateMapStack(nullptr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
    SrsPlannerPotentials::SrsPlannerPotentials(string name, costmap_2d::Costmap2DROS* rosCostMap) :
        srsMapStack_(nullptr),
        costmap_(NULL),
        initialized_(false),
        allow_unknown_(true)
{
    ROS_WARN("SrsPlanner::SrsPlanner(...) called");

    initialize(name, rosCostMap);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
    SrsPlannerPotentials::~SrsPlannerPotentials()
{
    delete astarCore_;
    delete srsMapStack_;

//    if (p_calc_)
//        delete p_calc_;
//    if (planner_)
//        delete planner_;
//    if (path_maker_)
//        delete path_maker_;
//    if (dsrv_)
//        delete dsrv_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::initialize(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
{
    if (!initialized_)
    {
        ROS_WARN("SrsPlanner::initialize(...) called");

        initializeParams();
        //### updateMapStack(costmap);

        costmap_ = costmap;
        frame_id_ = frame_id;

//        unsigned int cx = costmap->getSizeInCellsX();
//        unsigned int cy = costmap->getSizeInCellsY();

//        private_nh.param("old_navfn_behavior", old_navfn_behavior_, false);
//        if(!old_navfn_behavior_)
//            convert_offset_ = 0.5;
//        else
//            convert_offset_ = 0.0;

//        bool use_quadratic;
//        private_nh.param("use_quadratic", use_quadratic, true);
//        if (use_quadratic)
//             p_calc_ = new QuadraticCalculator(cx, cy);
//        else
//            p_calc_ = new PotentialCalculator(cx, cy);

//        bool use_dijkstra;
//        private_nh.param("use_dijkstra", use_dijkstra, true);
//        if (use_dijkstra)
//        {
//            DijkstraExpansion* de = new DijkstraExpansion(p_calc_, cx, cy);
//            if(!old_navfn_behavior_)
//                de->setPreciseStart(true);
//            planner_ = de;
//        }
//        else
//            planner_ = new AStarExpansion(p_calc_, cx, cy);

//        bool use_grid_path;
//        private_nh.param("use_grid_path", use_grid_path, false);
//        if (use_grid_path)
//            path_maker_ = new GridPath(p_calc_);
//        else
//            path_maker_ = new GradientPath(p_calc_);

        orientation_filter_ = new OrientationFilter();

        ros::NodeHandle private_nh("~/" + name);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potential_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

//        private_nh.param("allow_unknown", allow_unknown_, true);
//        planner_->setHasUnknown(allow_unknown_);

        private_nh.param("planner_window_x", planner_window_x_, 0.0);
        private_nh.param("planner_window_y", planner_window_y_, 0.0);
        private_nh.param("default_tolerance", default_tolerance_, 0.0);
        private_nh.param("publish_scale", publish_scale_, 100);

//        double costmap_pub_freq;
//        private_nh.param("planner_costmap_publish_frequency", costmap_pub_freq, 0.0);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

//        make_plan_srv_ = private_nh.advertiseService("make_plan", &GlobalPlanner::makePlanService, this);
//
//        dsrv_ = new dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>(ros::NodeHandle("~/" + name));
//        dynamic_reconfigure::Server<global_planner::GlobalPlannerConfig>::CallbackType cb = boost::bind(
//                &GlobalPlanner::reconfigureCB, this, _1, _2);
//        dsrv_->setCallback(cb);

        potential_array_ = nullptr;

        delete logicalMap_;

        LogicalMapFactory logicalMapFactory;
        logicalMap_ = logicalMapFactory.fromCostMap2D(costmap);

        astarCore_ = new AStarCore(logicalMap_, costmap);
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

//    double wx = ;
//    double wy = start.pose.position.y;

//    unsigned int start_x_i;
//    unsigned int start_y_i;
//    if (!costmap_->worldToMap(wx, wy, start_x_i, start_y_i))
//    {
//        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
//        return false;
//    }

//    double start_x;
//    double start_y;
//    worldToMap(wx, wy, start_x, start_y);

//    wx = ;
//    wy = goal.pose.position.y;

//    unsigned int goal_x_i;
//    unsigned int goal_y_i;
//    if (!costmap_->worldToMap(wx, wy, goal_x_i, goal_y_i))
//    {
//        ROS_WARN_THROTTLE(1.0, "The goal sent to the global planner is off the global costmap. Planning will always fail to this goal.");
//        return false;
//    }

//    double goal_x;
//    double goal_y;
//    worldToMap(wx, wy, goal_x, goal_y);

    // clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);

//    clearRobotCell(start_pose, start_x_i, start_y_i);

    std::vector<std::pair<float, float>> plan;
    bool found = astarCore_->calculatePath(
        start.pose.position.x, start.pose.position.y,
        goal.pose.position.x, goal.pose.position.y,
        plan,
        potential_array_);

//    int nx = costmap_->getSizeInCellsX(), ny = costmap_->getSizeInCellsY();
//
//    //make sure to resize the underlying array that Navfn uses
//    p_calc_->setSize(nx, ny);
//    planner_->setSize(nx, ny);
//    path_maker_->setSize(nx, ny);
//    potential_array_ = new float[nx * ny];
//
//    outlineMap(costmap_->getCharMap(), nx, ny, costmap_2d::LETHAL_OBSTACLE);
//
//    bool found_legal = planner_->calculatePotentials(costmap_->getCharMap(), start_x, start_y, goal_x, goal_y,
//                                                    nx * ny * 2, potential_array_);

//    planner_->clearEndpoint(costmap_->getCharMap(), potential_array_, goal_x_i, goal_y_i, 2);

    if (publish_potential_)
    {
        publishPotential(potential_array_);
    }

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
        ROS_ERROR("Failed to get a plan.");
    }

    delete potential_array_;
    potential_array_ = nullptr;

    return !path.empty();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::initializeParams()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::updateMapStack(costmap_2d::Costmap2DROS* rosCostMap)
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

//////////////////////////////////////////////////////////////////////////////////////////////////////
//void SrsPlannerPotentials::outlineMap(unsigned char* costarr, int nx, int ny, unsigned char value)
//{
//    unsigned char* pc = costarr;
//    for (int i = 0; i < nx; i++)
//        *pc++ = value;
//    pc = costarr + (ny - 1) * nx;
//    for (int i = 0; i < nx; i++)
//        *pc++ = value;
//    pc = costarr;
//    for (int i = 0; i < ny; i++, pc += nx)
//        *pc = value;
//    pc = costarr + nx - 1;
//    for (int i = 0; i < ny; i++, pc += nx)
//        *pc = value;
//}
//
//void SrsPlannerPotentials::reconfigureCB(global_planner::GlobalPlannerConfig& config, uint32_t level) {
//    planner_->setLethalCost(config.lethal_cost);
//    path_maker_->setLethalCost(config.lethal_cost);
//    planner_->setNeutralCost(config.neutral_cost);
//    planner_->setFactor(config.cost_factor);
//    publish_potential_ = config.publish_potential;
//    orientation_filter_->setMode(config.orientation_mode);
//}

//void SrsPlannerPotentials::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my)
//{
//    if (!initialized_) {
//        ROS_ERROR(
//                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
//        return;
//    }
//
//    //set the associated costs in the cost map to be free
//    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
//}

//bool SrsPlannerPotentials::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp) {
//    makePlan(req.start, req.goal, resp.plan.poses);
//
//    resp.plan.header.stamp = ros::Time::now();
//    resp.plan.header.frame_id = frame_id_;
//
//    return true;
//}

void SrsPlannerPotentials::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if (!initialized_)
    {
        ROS_ERROR(
                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

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

void SrsPlannerPotentials::getPlanFromPotential(
    std::vector<std::pair<float, float>>& path,
//    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
//    std::string global_frame = frame_id_;

    plan.clear();

//    std::vector<std::pair<float, float> > path;
//
//    if (!path_maker_->getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
//        ROS_ERROR("NO PATH!");
//        return false;
//    }
//
    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() - 1; i>=0; i--)
    {
        std::pair<float, float> point = path[i];

        cout << point.first << "-" << point.second << endl;

        double world_x;
        double world_y;
        astarCore_->mapToWorld(point.first, point.second, world_x, world_y);

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

} // namespace srs


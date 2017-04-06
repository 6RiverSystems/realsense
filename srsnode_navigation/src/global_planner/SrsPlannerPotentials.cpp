#include <srsnode_navigation/global_planner/SrsPlannerPotentials.hpp>

#include <pluginlib/class_list_macros.h>

#include <global_planner/planner_core.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>

#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/mapnote/NoteQueue.hpp>
#include <srslib_framework/math/MeasurementMath.hpp>

#include <srsnode_navigation/global_planner/potentials/QuadraticCalculator.hpp>
#include <srsnode_navigation/global_planner/potentials/GradientPath.hpp>
#include <srsnode_navigation/global_planner/potentials/AStarExpansion.hpp>

PLUGINLIB_EXPORT_CLASS(srs::SrsPlannerPotentials, nav_core::BaseGlobalPlanner)

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlannerPotentials::SrsPlannerPotentials() :
    srsMapStack_(nullptr),
    costMap_(nullptr),
    initialized_(false),
    astar_(nullptr),
    useQuadratic_(true),
    useGridPath_(true),
    allowUnknown_(true),
    publishPotential_(true),
    lethalCost_(253),
    neutralCost_(50)
{
    ROS_WARN("SrsPlannerPotentials::SrsPlannerPotentials() called");

    updateMapStack(nullptr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlannerPotentials::SrsPlannerPotentials(string name, costmap_2d::Costmap2DROS* rosCostMap) :
    srsMapStack_(nullptr),
    costMap_(nullptr),
    initialized_(false),
    astar_(nullptr),
    useQuadratic_(true),
    useGridPath_(true),
    allowUnknown_(true),
    publishPotential_(true),
    lethalCost_(253),
    neutralCost_(50)
{
    ROS_WARN("SrsPlannerPotentials::SrsPlannerPotentials(...) called");

    initialize(name, rosCostMap);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlannerPotentials::~SrsPlannerPotentials()
{
    delete configServer_;
    delete srsMapStack_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::initialize(std::string name, costmap_2d::Costmap2D* costmap,
    std::string frame_id)
{
    if (!initialized_)
    {
        ROS_WARN("SrsPlannerPotentials::initialize(...) called");

        orientationFilter_ = new OrientationFilter();


        ros::NodeHandle private_nh("~/" + name);
        configServer_ = new dynamic_reconfigure::Server<srsnode_navigation::SrsPlannerConfig>(private_nh);
        configServer_->setCallback(boost::bind(&SrsPlannerPotentials::onConfigChange, this, _1, _2));

        updateMapStack(costmap);

        costMap_ = costmap;
        tfFrameid_ = frame_id;


        planPublisher_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        potentialPublisher_ = private_nh.advertise<nav_msgs::OccupancyGrid>("potential", 1);

        private_nh.param("publish_scale", publishScale_, 100);

        ros::NodeHandle prefix_nh;
        tfPrefix_ = tf::getPrefixParam(prefix_nh);

        potentialArray_ = nullptr;
        initialized_ = true;
    }
    else
    {
        ROS_WARN("Planner has already been initialized");
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
    ROS_DEBUG("SrsPlanner::makePlan() called");

    // Make sure that we get the latest Map Stack
    updateMapStack(nullptr);

    return makePlan(start, goal, 0, plan);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool SrsPlannerPotentials::makePlan(
    const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
    double tolerance, std::vector<geometry_msgs::PoseStamped>& path)
{
    boost::mutex::scoped_lock lock(mutex_);

    if (!initialized_)
    {
        ROS_ERROR("This planner has not been initialized yet. Please call initialize() first");
        return false;
    }

    path.clear();

    if (tf::resolve(tfPrefix_, goal.header.frame_id) != tf::resolve(tfPrefix_, tfFrameid_))
    {
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
            tf::resolve(tfPrefix_, tfFrameid_).c_str(), tf::resolve(tfPrefix_, goal.header.frame_id).c_str());
        return false;
    }

    if (tf::resolve(tfPrefix_, start.header.frame_id) != tf::resolve(tfPrefix_, tfFrameid_))
    {
        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.",
            tf::resolve(tfPrefix_, tfFrameid_).c_str(), tf::resolve(tfPrefix_, start.header.frame_id).c_str());
        return false;
    }

    astar_ = new AStarPotentials(srsMapStack_->getLogicalMap(), costMap_, queuesMap_);

    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);

    std::vector<std::pair<float, float>> plan;

    AStarPotentials::SearchParameters searchParams;
    searchParams.useQuadratic = useQuadratic_;
    searchParams.useGridPath = useGridPath_;
    searchParams.allowUnknown = allowUnknown_;
    searchParams.lethalCost = lethalCost_;
    searchParams.neutralCost = neutralCost_;
    searchParams.weightRatio = weightRatio_;
    searchParams.logicalCostRatio = logicalCostRatio_;

    bool found = astar_->calculatePath(searchParams,
        start.pose.position.x, start.pose.position.y,
        goal.pose.position.x, goal.pose.position.y,
        plan,
        potentialArray_);

    if (publishPotential_)
    {
        publishPotential(potentialArray_);
    }

    if (found)
    {
        getPlanFromPotential(plan, path);

        geometry_msgs::PoseStamped goal_copy = goal;
        goal_copy.header.stamp = ros::Time::now();
        path.push_back(goal_copy);

        orientationFilter_->processPath(start, path);

        publishPlan(path);
    }
    else
    {
        ROS_ERROR("Failed to get a plan. Saving map.");
        costMap_->saveMap("cost_map.pgm");
    }

    delete potentialArray_;
    potentialArray_ = nullptr;

    delete astar_;
    astar_ = nullptr;

    return !path.empty();
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::extractQueuePolygons()
{
    queuesMap_.clear();

    LogicalMap* logicalMap = srsMapStack_->getLogicalMap();

    // Go through all the areas and find the queues
    for (auto area : logicalMap->getAreas())
    {
        LogicalMap::LabeledArea queue = area.second;

        shared_ptr<NoteQueue> note = queue.notes->get<NoteQueue>(NoteQueue::TYPE);
        if (note)
        {
            // Convert the queue region into map coordinates
            double x0;
            double y0;
            double x1;
            double y1;
            logicalMap->transformCells2M(queue.surface.x0, queue.surface.y0, x0, y0);
            logicalMap->transformCells2M(queue.surface.x1, queue.surface.y1, x1, y1);

            // Create the polygon that represents the region of space of the queue
            AStarPotentials::PolygonType polygon;

            geometry_msgs::Point p1;
            p1.x = x0;
            p1.y = y0;
            p1.z = 0;
            polygon.push_back(p1);

            geometry_msgs::Point p2;
            p2.x = x1;
            p2.y = y0;
            p2.z = 0;
            polygon.push_back(p2);

            geometry_msgs::Point p3;
            p3.x = x1;
            p3.y = y1;
            p3.z = 0;
            polygon.push_back(p3);

            geometry_msgs::Point p4;
            p4.x = x0;
            p4.y = y1;
            p4.z = 0;
            polygon.push_back(p4);

            queuesMap_.insert({queue.label, polygon});
        }
    }
}

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
        pose.header.frame_id = tfFrameid_;
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
void SrsPlannerPotentials::onConfigChange(srsnode_navigation::SrsPlannerConfig& config, uint32_t level)
{
    useQuadratic_ = config.use_quadratic;
    useGridPath_ = config.use_grid_path;
    allowUnknown_ = config.allow_unknown;
    lethalCost_ = config.lethal_cost;
    neutralCost_ = config.neutral_cost;
    publishPotential_ = config.publish_potential;
    weightRatio_ = config.weight_ratio;
    logicalCostRatio_ = config.logical_cost_ratio;
    orientationFilter_->setMode(config.orientation_mode);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
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

    planPublisher_.publish(gui_path);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerPotentials::publishPotential(float* potential)
{
    int nx = costMap_->getSizeInCellsX();
    int ny = costMap_->getSizeInCellsY();

    double resolution = costMap_->getResolution();

    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = tfFrameid_;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;
    grid.info.origin.position.x = costMap_->getOriginX();
    grid.info.origin.position.y = costMap_->getOriginY();
    grid.info.width = nx;
    grid.info.height = ny;

    double wx;
    double wy;
    costMap_->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(nx * ny);

    float max = 0.0;
    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        float potential = potentialArray_[i];
        if (potential < PotentialCalculator::MAX_POTENTIAL)
        {
            if (potential > max)
            {
                max = potential;
            }
        }
    }

    for (unsigned int i = 0; i < grid.data.size(); i++)
    {
        if (potentialArray_[i] >= PotentialCalculator::MAX_POTENTIAL)
        {
            grid.data[i] = -1;
        }
        else
        {
            grid.data[i] = potentialArray_[i] * publishScale_ / max;
        }
    }

    potentialPublisher_.publish(grid);
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

        extractQueuePolygons();
    }
}

} // namespace srs


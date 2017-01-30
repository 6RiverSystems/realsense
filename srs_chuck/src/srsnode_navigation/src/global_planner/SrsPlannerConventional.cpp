#include <srsnode_navigation/global_planner/SrsPlannerConventional.hpp>

#include <pthread.h>

#include <pluginlib/class_list_macros.h>

#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/SimpleTrajectoryGenerator.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/Trajectory.hpp>
#include <srslib_timing/StopWatch.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

PLUGINLIB_EXPORT_CLASS(srs::SrsPlannerConventional, nav_core::BaseGlobalPlanner)

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlannerConventional::SrsPlannerConventional() :
    srsMapStack_(nullptr),
    costMap2d_(nullptr)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlannerConventional::SrsPlannerConventional(string name, costmap_2d::Costmap2DROS* rosCostMap) :
    srsMapStack_(nullptr),
    costMap2d_(nullptr)
{
    initialize(name, rosCostMap);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlannerConventional::~SrsPlannerConventional()
{
    delete srsMapStack_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerConventional::initialize(std::string name, costmap_2d::Costmap2DROS* rosCostMap)
{
    ros::NodeHandle privateNh("~/" + name);

    costMap2d_ = rosCostMap->getCostmap();
    tapMapStack_.attach(this);

    configServer_ = new dynamic_reconfigure::Server<srsnode_navigation::SrsPlannerConfig>(privateNh);
    configServer_->setCallback(boost::bind(&SrsPlannerConventional::onConfigChange, this, _1, _2));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool SrsPlannerConventional::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    vector<geometry_msgs::PoseStamped>& plan)
{
    // Find a suitable solution for the provided goal
    Pose<> robotPose = PoseMessageFactory::poseStamped2Pose(start);
    Pose<> target = PoseMessageFactory::poseStamped2Pose(goal);

    StopWatch watch;
    Solution<Grid2dSolutionItem>* solution = Grid2dSolutionFactory::fromSingleGoal(
        srsMapStack_, robotPose, target, astarConfigParameters_, nodeSearchParameters_);

    ROS_INFO_STREAM_NAMED("srs_planner", "Global planner elpased time: " <<
        watch.elapsedMilliseconds() << "ms");

    plan.clear();

    if (solution && solution->isValid())
    {
        ROS_DEBUG_STREAM_NAMED("srs_planner", "Found solution: " << endl << *solution);

        // Calculate the trajectory from the solution found by A*,
        // using the default Chuck model
        SimpleTrajectoryGenerator converter(srsMapStack_);
        converter.fromSolution(solution);

        Trajectory<> trajectory;
        converter.getTrajectory(trajectory);

        ROS_DEBUG_STREAM_NAMED("srs_planner", "Trajectory: " << trajectory);

        channelRosPath_.publish(trajectory);
        populatePath(start, trajectory, goal, plan);

        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerConventional::notified(Subscriber<srslib_framework::MapStack>* subject)
{
    ROS_DEBUG_STREAM_NAMED("srs_planner", "SrsPlanner notified with a new Map Stack");

    TapMapStack* tapMapStack = static_cast<TapMapStack*>(subject);

    // When the Map Stack is published, make sure to get a fresh copy
    delete srsMapStack_;
    srsMapStack_ = tapMapStack_.pop();

    // Include the ROS costmap in the map stack
    srsMapStack_->setCostMap2d(costMap2d_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerConventional::onConfigChange(srsnode_navigation::SrsPlannerConfig& config, uint32_t level)
{
    astarConfigParameters_.useYield = config.use_yield;
    astarConfigParameters_.yieldFrequency = config.yield_frequency;

    nodeSearchParameters_.allowUnknown = config.allow_unknown;
    nodeSearchParameters_.costMapRatio = config.cost_map_ratio;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlannerConventional::populatePath(const geometry_msgs::PoseStamped& start,
    const Trajectory<>& trajectory,
    const geometry_msgs::PoseStamped& goal,
    vector<geometry_msgs::PoseStamped>& plan)
{
    plan.push_back(start);

    for (auto waypoint : trajectory)
    {
        Pose<> currentPose = waypoint.first;

        // Create the stamped pose and the quaternion for the
        // trajectory step
        tf::Quaternion quaternion = tf::createQuaternionFromYaw(currentPose.theta);
        geometry_msgs::PoseStamped step = PoseMessageFactory::pose2PoseStamped(currentPose);

        step.pose.position.x = currentPose.x;
        step.pose.position.y = currentPose.y;
        step.pose.orientation.x = quaternion.x();
        step.pose.orientation.y = quaternion.y();
        step.pose.orientation.z = quaternion.z();
        step.pose.orientation.w = quaternion.w();

        plan.push_back(step);
    }

    plan.push_back(goal);
}

} // namespace srs

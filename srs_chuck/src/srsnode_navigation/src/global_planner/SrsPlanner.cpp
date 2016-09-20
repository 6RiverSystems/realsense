#include <srsnode_navigation/global_planner/SrsPlanner.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(srs::SrsPlanner, nav_core::BaseGlobalPlanner)

#include <srslib_framework/localization/map/MapFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridTrajectoryGenerator.hpp>
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlanner::SrsPlanner() :
    srsMap_(nullptr)
{
    ROS_DEBUG("SrsPlanner::SrsPlanner() called");

    initializeParams();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlanner::SrsPlanner(string name, costmap_2d::Costmap2DROS* rosCostMap) :
    srsMap_(nullptr)
{
    ROS_DEBUG("SrsPlanner::SrsPlanner(...) called");

    initializeParams();
    initialize(name, rosCostMap);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlanner::~SrsPlanner()
{
    delete srsMap_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* rosCostMap)
{
    ROS_DEBUG_STREAM("SrsPlanner::initialize() called");

    delete srsMap_;
    srsMap_ = MapFactory::fromRosCostMap2D(rosCostMap, weightObstacleThreshold_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool SrsPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    vector<geometry_msgs::PoseStamped>& plan)
{
    ROS_WARN("SrsPlanner::makePlan() called");

    // Find a suitable solution for the provided goal
    Pose<> robotPose = PoseMessageFactory::poseStamped2Pose(start);
    Pose<> target = PoseMessageFactory::poseStamped2Pose(goal);
    Solution<GridSolutionItem>* solution = GridSolutionFactory::fromGoal(srsMap_, robotPose, target);

    plan.clear();

    if (solution && !solution->empty())
    {
        ROS_DEBUG_STREAM_NAMED("srs_planner", "Found solution: " << endl << *solution);

        Trajectory<> trajectory;

        // Calculate the trajectory from the solution found by A*,
        // using the default Chuck model

        // TODO: Remove this assumption
        Chuck chuck;
        GridTrajectoryGenerator converter(chuck);
        converter.fromSolution(*solution);
        converter.getTrajectory(trajectory);

        ROS_DEBUG_STREAM_NAMED("srs_planner", "Trajectory: " << trajectory);

        plan.push_back(start);
        for (auto element : trajectory)
        {
            Pose<> currentPose = element.first;

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

        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

void SrsPlanner::initializeParams()
{
    ros::NodeHandle private_nh;
    bool inflationEnabled;

    private_nh.param("/move_base/global_costmap/inflation_layer/cost_scaling_factor",
        weightScaleFactor_, 1.0);
    private_nh.param("/move_base/global_costmap/inflation_layer/enabled",
        inflationEnabled, true);

    weightObstacleThreshold_ = round((100.0 * (inflationEnabled ? weightScaleFactor_ : 1.0)) * 0.98);
}

} // namespace srs

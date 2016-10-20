#include <srsnode_navigation/global_planner/SrsPlanner.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(srs::SrsPlanner, nav_core::BaseGlobalPlanner)

#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dTrajectoryGenerator.hpp>
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlanner::SrsPlanner() :
    srsMapStack_(nullptr)
{
    ROS_WARN("SrsPlanner::SrsPlanner() called");

    initializeParams();
    updateMapStack(nullptr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlanner::SrsPlanner(string name, costmap_2d::Costmap2DROS* rosCostMap) :
    srsMapStack_(nullptr)
{
    ROS_WARN("SrsPlanner::SrsPlanner(...) called");

    initialize(name, rosCostMap);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlanner::~SrsPlanner()
{
    delete srsMapStack_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* rosCostMap)
{
    ROS_WARN("SrsPlanner::initialize() called");

    initializeParams();
    updateMapStack(rosCostMap);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool SrsPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    vector<geometry_msgs::PoseStamped>& plan)
{
    ROS_WARN("SrsPlanner::makePlan() called");

    // Make sure that we get the latest Map Stack
    updateMapStack(nullptr);

    // Find a suitable solution for the provided goal
    Pose<> robotPose = PoseMessageFactory::poseStamped2Pose(start);
    Pose<> target = PoseMessageFactory::poseStamped2Pose(goal);

    Solution<Grid2dSolutionItem>* solution = Grid2dSolutionFactory::fromSingleGoal(
        srsMapStack_->getLogicalMap(), robotPose, target);

    plan.clear();

    if (solution && !solution->empty())
    {
        ROS_DEBUG_STREAM_NAMED("srs_planner", "Found solution: " << endl << *solution);

        Trajectory<> trajectory;

        // Calculate the trajectory from the solution found by A*,
        // using the default Chuck model

        // TODO: Remove this assumption
        Chuck chuck;
        Grid2dTrajectoryGenerator converter(chuck);
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

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlanner::initializeParams()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlanner::updateMapStack(costmap_2d::Costmap2DROS* rosCostMap)
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

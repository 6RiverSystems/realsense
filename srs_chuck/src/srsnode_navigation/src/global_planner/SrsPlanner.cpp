#include <srsnode_navigation/global_planner/SrsPlanner.hpp>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(srs::SrsPlanner, nav_core::BaseGlobalPlanner)

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlanner::SrsPlanner(string name, costmap_2d::Costmap2DROS* costmap_ros) :
    srsMap_(nullptr)
{
    initialize(name, costmap_ros);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
SrsPlanner::~SrsPlanner()
{
    delete srsMap_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void SrsPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    ROS_WARN("SrsPlanner::initialize called");

    delete srsMap_;
    srsMap_ = nullptr;

    if (costmap_ros)
    {
        srsMap_ = new Map(costmap_ros->getCostmap());
    }

    cout << *srsMap_ << endl;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool SrsPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    vector<geometry_msgs::PoseStamped>& plan)
{
    ROS_WARN("SrsPlanner::makePlan called");
    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

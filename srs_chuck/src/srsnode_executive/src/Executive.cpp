#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/PoseAdapter.hpp>
#include <srslib_framework/robotics/robot_profile/ChuckProfile.hpp>
#include <srslib_framework/ros/message/SolutionMessageFactory.hpp>
#include <srslib_framework/ros/service/RosCallEmpty.hpp>
#include <srslib_framework/ros/service/RosCallSetBool.hpp>
#include <srslib_framework/ros/service/RosCallSolution.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive(string name, int argc, char** argv) :
    RosUnit(name, argc, argv, REFRESH_RATE_HZ)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::execute()
{
    updateRobotPose();

    labeledAreasDetector_.evaluatePose(robotPose_);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::initialize()
{
    // Nothing to initialize
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::findActiveNodes(vector<string>& nodes)
{
    nodes.clear();

    ros::V_string rosMasterNodes;
    ros::master::getNodes(rosMasterNodes);

    string nameSpace = rosNodeHandle_.getNamespace();

    for (auto node : rosMasterNodes)
    {
        if (node.find("srsnode") != string::npos &&
            node.find(nameSpace) == string::npos)
        {
            nodes.push_back(node);
            ROS_INFO_STREAM(node);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::updateRobotPose()
{
    robotPose_ = tapRobotPose_.pop();
}

} // namespace srs

#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/PolygonStamped.h>

#include <srslib_framework/Pose.h>
#include <srslib_framework/MsgSolution.h>

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
    try
    {
        tf::StampedTransform robotTransform;
        tfListener_.lookupTransform("/map", "/base_footprint", ros::Time(0), robotTransform);

        tf::Vector3 location = robotTransform.getOrigin();
        tf::Quaternion orientation = robotTransform.getRotation();
        robotPose_ = Pose<>(location.getX(), location.getY(), tf::getYaw(orientation));

        ROS_DEBUG_STREAM_THROTTLE_NAMED(1.0, "executive", "Robot pose" << robotPose_);
    }
    catch(const tf::TransformException& e)
    {
        ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, "executive", "TF Exception: " << e.what());
    }

}


} // namespace srs

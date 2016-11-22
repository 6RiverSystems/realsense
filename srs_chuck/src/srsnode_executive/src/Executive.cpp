#include <srsnode_executive/Executive.hpp>

#include <ros/ros.h>

#include <srsnode_executive/condition/IsEnteringWarningSoundArea.hpp>
#include <srsnode_executive/condition/IsExitingWarningSoundArea.hpp>
#include <srsnode_executive/task/PlayWarningSound.hpp>
#include <srsnode_executive/task/FindLabeledAreas.hpp>
#include <srsnode_executive/task/StopSound.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Executive::Executive(string name, int argc, char** argv) :
    RosUnit(name, argc, argv, REFRESH_RATE_HZ)
{
    mainSequence_ = Sequence<ExecutiveContext>({
        new FindLabeledAreas(),
        new PlayWarningSound(new IsEnteringWarningSoundArea())
    });

    context_.robotPose = Pose<>::INVALID;
    context_.mapStack = nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Executive::execute()
{
    updateContext();

    mainSequence_.execute(&context_);
//    playWarningSound_.execute(&context_);
//    stopSound_.execute(&context_);

    // labeledAreasDetector_.evaluatePose(context_.robotPose);
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
void Executive::updateContext()
{
    context_.robotPose = tapRobotPose_.pop();

    // Make sure that the neither the logical not the occupancy maps
    // have been re-published. In case, destroy what we have and
    // ask for a new stack
    if (tapMapStack_.newDataAvailable())
    {
        context_.mapStack = tapMapStack_.pop();
    }
}

} // namespace srs

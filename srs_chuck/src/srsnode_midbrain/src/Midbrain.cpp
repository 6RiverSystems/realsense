#include <srsnode_midbrain/Midbrain.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Midbrain::Midbrain(string nodeName) :
    rosNodeHandle_(nodeName),
    triggerPause_(rosNodeHandle_),
    triggerShutdown_(rosNodeHandle_)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Midbrain::run()
{
    triggerShutdown_.connectService();
    triggerPause_.connectService();

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        evaluateTriggers();

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Midbrain::evaluateTriggers()
{
    if (triggerPause_.isTriggerRequested())
    {
    }

    if (triggerShutdown_.isTriggerRequested())
    {
        ros::shutdown();
    }
}

} // namespace srs

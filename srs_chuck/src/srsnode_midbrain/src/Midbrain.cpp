/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Midbrain.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Midbrain::Midbrain(string nodeName) :
    rosNodeHandle_(nodeName),
    reflexes_(rosNodeHandle_)
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

        reflexes_.PublishDangerZone();

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void Midbrain::evaluateTriggers()
{
//    if (triggerPause_.isTriggerRequested())
//    {
//    }

    if (triggerShutdown_.isTriggerRequested())
    {
        ros::shutdown();
    }
}

} // namespace srs

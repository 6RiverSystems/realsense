/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSSERVICETRIGGER_HPP_
#define ROSSERVICETRIGGER_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <srslib_framework/ros/RosService.hpp>

namespace srs {

class RosServiceTrigger :
    public RosService
{
public:
    RosServiceTrigger(ros::NodeHandle rosHandle, string serviceName, string triggerName) :
        RosService(rosHandle, serviceName),
        triggerName_("trigger/" + triggerName),
        triggerRequested_(false)
    {
    }

    ~RosServiceTrigger()
    {}

    bool isTriggerRequested()
    {
        if (newRequestPending())
        {
            setNewRequest(false);
            return triggerRequested_;
        }
        return false;
    }

protected:
    bool connect()
    {
        rosServiceServer_ = rosNodeHandle_.advertiseService(triggerName_,
            &RosServiceTrigger::onTriggerRequested, this);

        return true;
    }

private:
    string triggerName_;
    bool triggerRequested_;

    bool onTriggerRequested(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
    {
        triggerRequested_ = true;
        setNewRequest(true);

        return true;
    }
};

} // namespace srs

#endif // ROSSERVICETRIGGER_HPP_

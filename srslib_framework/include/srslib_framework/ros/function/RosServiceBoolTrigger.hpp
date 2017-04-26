/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSSERVICEBOOLTRIGGER_HPP_
#define ROSSERVICEBOOLTRIGGER_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <srslib_framework/ros/service/RosService.hpp>

namespace srs {

class RosServiceBoolTrigger :
    public RosService
{
public:
    RosServiceBoolTrigger(string name, string description = "") :
        RosService(name, description),
        triggerName_("trigger/" + name),
        serviceValue_(false)
    {}

    ~RosServiceBoolTrigger()
    {}

    bool getRequest()
    {
        setNewRequest(false);
        return serviceValue_;
    }

protected:
    bool connect()
    {
        rosServiceServer_ = rosNodeHandle_.advertiseService(triggerName_,
            &RosServiceBoolTrigger::onServiceRequested, this);

        return true;
    }

private:
    string triggerName_;
    bool serviceValue_;

    bool onServiceRequested(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& resp)
    {
        serviceValue_ = req.data;
        setNewRequest(true);

        return true;
    }
};

} // namespace srs

#endif // ROSSERVICEBOOLTRIGGER_HPP_

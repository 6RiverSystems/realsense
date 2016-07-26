/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSSERVICEEXECUTESOLUTION_HPP_
#define ROSSERVICEEXECUTESOLUTION_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <srslib_framework/ExecuteSolution.h>

#include <srslib_framework/ros/RosService.hpp>
#include <srslib_framework/ros/message/SolutionMessageFactory.hpp>

namespace srs {

class RosServiceExecuteSolution :
    public RosService
{
public:
    RosServiceExecuteSolution() :
        RosService("execute_solution", "Trigger: Execute Solution"),
        triggerName_("trigger/execute_solution")
    {}

    ~RosServiceExecuteSolution()
    {}

    Solution<GridSolutionItem> getRequest()
    {
        setNewRequest(false);
        return serviceValue_;
    }

protected:
    bool connect()
    {
        rosServiceServer_ = rosNodeHandle_.advertiseService(triggerName_,
            &RosServiceExecuteSolution::onServiceRequested, this);

        return true;
    }

private:
    string triggerName_;
    Solution<GridSolutionItem> serviceValue_;

    bool onServiceRequested(srslib_framework::ExecuteSolution::Request& req,
        srslib_framework::ExecuteSolution::Response& resp)
    {
        serviceValue_ = SolutionMessageFactory::msg2Solution(req.solution);
        setNewRequest(true);

        resp.result = true;

        return true;
    }
};

} // namespace srs

#endif // ROSSERVICEEXECUTESOLUTION_HPP_

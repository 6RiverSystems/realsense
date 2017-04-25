/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srsnode_executive/task/Task.hpp>

namespace srs {

class TaskSetMaxVelocity : public Task
{
public:
    TaskSetMaxVelocity() :
        defaultMaxVelocity_(1.0)
    {}

    virtual ~TaskSetMaxVelocity()
    {}

    void setDefaultMaxVelocity(float maxVelocity)
    {
        defaultMaxVelocity_ = maxVelocity;
    }

    void run(ExecutiveContext& context);

private:
    float defaultMaxVelocity_;
};

} // namespace srs

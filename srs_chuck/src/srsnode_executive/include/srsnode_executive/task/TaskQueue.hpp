/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srsnode_executive/task/Task.hpp>

namespace srs {

class TaskQueue : public Task
{
public:
    TaskQueue()
    {}

    virtual ~TaskQueue()
    {}

    void run(ExecutiveContext& context);

    void setDefaultRealsenseDecayTime(float defaultRealsenseDecayTime)
    {
        defaultRealsenseDecayTime_ = defaultRealsenseDecayTime;
    }

private:
    float defaultRealsenseDecayTime_;
};

} // namespace srs

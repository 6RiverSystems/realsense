/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srsnode_executive/task/Task.hpp>

namespace srs {

class TaskStayOnPath : public Task
{
public:
    TaskStayOnPath()
    {}

    virtual ~TaskStayOnPath()
    {}

    void run(ExecutiveContext& context);
};

} // namespace srs

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srsnode_executive/ExecutiveContext.hpp>

namespace srs {

class Task
{
public:
    Task()
    {}

    virtual ~Task()
    {}

    virtual void run(ExecutiveContext& context) = 0;
};

} // namespace srs

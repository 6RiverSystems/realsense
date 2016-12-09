/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srsnode_executive/task/Task.hpp>

namespace srs {

class TaskDetectLabeledAreas : public Task
{
public:
    TaskDetectLabeledAreas()
    {}

    virtual ~TaskDetectLabeledAreas()
    {}

    void run(ExecutiveContext& context);

private:
    bool doesExist(vector<ExecutiveContext::ActiveLabelType>& areaList, string label);
};

} // namespace srs

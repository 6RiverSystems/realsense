/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/behavior/behavior_tree/TreeNode.hpp>
#include <srslib_framework/behavior/behavior_tree/Task.hpp>

#include <srsnode_executive/ExecutiveContext.hpp>

namespace srs {

class FindLabeledAreas :
    public Task<ExecutiveContext>
{
public:
    FindLabeledAreas() :
        Task<ExecutiveContext>()
    {}

    virtual ~FindLabeledAreas()
    {}

    TreeNode<ExecutiveContext>::NodeResult execute(ExecutiveContext* context);
private:
};

} // namespace srs

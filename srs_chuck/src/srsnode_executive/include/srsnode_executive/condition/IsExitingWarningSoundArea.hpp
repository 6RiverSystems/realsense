/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <nav_msgs/Odometry.h>

#include <srslib_framework/behavior/behavior_tree/TreeNode.hpp>
#include <srslib_framework/behavior/behavior_tree/Condition.hpp>

#include <srsnode_executive/ExecutiveContext.hpp>

namespace srs {

class IsExitingWarningSoundArea :
    public Condition<ExecutiveContext>
{
public:
    IsExitingWarningSoundArea()
    {}

    virtual ~IsExitingWarningSoundArea()
    {}

    TreeNode<ExecutiveContext>::NodeResult execute(ExecutiveContext* context);
};

} // namespace srs

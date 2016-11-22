/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/behavior/behavior_tree/TreeNodeWithCondition.hpp>

namespace srs {

template<class CONTEXT>
class Task :
    public TreeNodeWithCondition<CONTEXT>
{
public:
    Task(Condition<CONTEXT>* preCondition = nullptr) :
        TreeNodeWithCondition<CONTEXT>(preCondition)
    {}

    virtual ~Task()
    {}

    virtual typename TreeNode<CONTEXT>::NodeResult execute(CONTEXT* context);
};

} // namespace srs

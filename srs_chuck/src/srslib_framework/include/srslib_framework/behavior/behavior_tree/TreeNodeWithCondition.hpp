/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/behavior/behavior_tree/TreeNode.hpp>
#include <srslib_framework/behavior/behavior_tree/Condition.hpp>

namespace srs {

template<class CONTEXT>
class TreeNodeWithCondition :
    public TreeNode<CONTEXT>
{
public:
    using ResultType = typename TreeNode<CONTEXT>::NodeResult;

    TreeNodeWithCondition(Condition<CONTEXT>* preCondition = nullptr) :
        TreeNode<CONTEXT>(),
        preCondition_(preCondition)
    {}

    virtual ~TreeNodeWithCondition()
    {}

    virtual ResultType execute(CONTEXT* context);

protected:
    Condition<CONTEXT>* preCondition_;
};

} // namespace srs

#include <behavior/behavior_tree/TreeNodeWithCondition.cpp>

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/behavior/behavior_tree/TreeNode.hpp>

namespace srs {

template<class CONTEXT>
class Task :
    public TreeNode<CONTEXT>
{
public:
    using ResultType = typename TreeNodeWithCondition<CONTEXT>::ResultType;

    Task(Condition<CONTEXT>* preCondition = nullptr) :
        TreeNode<CONTEXT>(preCondition)
    {}

    virtual ~Task()
    {}

    virtual ResultType run(CONTEXT* context) = 0;

protected:
    virtual ResultType execute(CONTEXT* context)
    {
        Task::NodeResult conditionResult = Task::NodeResult::SUCCEDED;
        if (preCondition_)
        {
            conditionResult = preCondition_->evaluate(context);
        }

        if (conditionResult == Task::NodeResult::SUCCEDED)
        {
            return run(context);
        }

        return conditionResult;
    }
};

} // namespace srs

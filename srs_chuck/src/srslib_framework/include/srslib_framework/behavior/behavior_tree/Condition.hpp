/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/behavior/behavior_tree/TreeNode.hpp>

namespace srs {

template<class CONTEXT>
class Condition :
    public TreeNode<CONTEXT>
{
public:
    Condition()
    {}

    virtual ~Condition()
    {}

    virtual ResultType evaluate(CONTEXT* context) = 0;

protected:
    virtual ResultType execute(CONTEXT* context)
    {
        return evaluate(context);
    }
};

} // namespace srs

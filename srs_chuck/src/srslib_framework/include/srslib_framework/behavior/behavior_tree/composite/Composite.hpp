/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/behavior/behavior_tree/PreconditionedTreeNode.hpp>

namespace srs {

template<class CONTEXT>
class Composite :
    public PreconditionedTreeNode<CONTEXT>
{
public:
    Composite(Condition<CONTEXT>* preCondition = nullptr) :
        PreconditionedTreeNode<CONTEXT>(preCondition)
    {}

    virtual ~Composite()
    {}
};

} // namespace srs

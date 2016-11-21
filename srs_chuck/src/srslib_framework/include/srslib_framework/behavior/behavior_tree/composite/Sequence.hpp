/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
#include <initializer_list>
using namespace std;

#include <srslib_framework/behavior/behavior_tree/TreeNode.hpp>
#include <srslib_framework/behavior/behavior_tree/composite/Composite.hpp>

namespace srs {

template<class CONTEXT>
class Sequence :
    public Composite<CONTEXT>
{
public:
    using ResultType = typename Composite<CONTEXT>::ResultType;

    Sequence() :
        Composite<CONTEXT>()
    {}

    Sequence(initializer_list<TreeNode<CONTEXT>*> children) :
        Composite<CONTEXT>(children)
    {}

    virtual ~Sequence()
    {}

    virtual ResultType execute(CONTEXT* context);
};

} // namespace srs

#include <behavior/behavior_tree/composite/Sequence.cpp>

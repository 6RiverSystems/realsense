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
    Sequence() :
        Composite<CONTEXT>(nullptr)
    {}

    Sequence(initializer_list<TreeNode<CONTEXT>*> list) :
        Composite<CONTEXT>(nullptr)
    {
        children_ = list;
    }

    virtual ~Sequence()
    {}

    virtual typename TreeNode<CONTEXT>::NodeResult execute(CONTEXT* context);

private:
    vector<TreeNode<CONTEXT>*> children_;
};

} // namespace srs

#include <behavior/behavior_tree/composite/Sequence.cpp>

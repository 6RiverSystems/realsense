/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/behavior/behavior_tree/TreeNode.hpp>

namespace srs {

template<class CONTEXT>
class Composite :
    public TreeNode<CONTEXT>
{
public:
    using ResultType = typename TreeNode<CONTEXT>::NodeResult;

    Composite() :
        children_()
    {}

    Composite(initializer_list<TreeNode<CONTEXT>*> children) :
        children_(children)
    {}

    virtual ~Composite()
    {}

protected:
    vector<TreeNode<CONTEXT>*> children_;
};

} // namespace srs

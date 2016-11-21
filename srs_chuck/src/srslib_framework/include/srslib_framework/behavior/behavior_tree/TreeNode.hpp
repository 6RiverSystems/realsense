/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

template<class CONTEXT>
class TreeNode
{
public:
    enum NodeResult {
        FAILED,
        RUNNING,
        SUCCEDED
    };

    TreeNode()
    {}

    virtual ~TreeNode()
    {}

    virtual NodeResult execute(CONTEXT* context) = 0;
};

} // namespace srs

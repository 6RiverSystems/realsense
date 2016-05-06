/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SOLUTIONNODE_HPP_
#define SOLUTIONNODE_HPP_

#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/search/SearchAction.hpp>

namespace srs {

template<typename GRAPH>
struct SolutionNode
{
    SolutionNode() :
        action(SearchAction<GRAPH>::NONE)
    {}

    friend ostream& operator<<(ostream& stream, const SolutionNode& solutionNode)
    {
        return stream << " (" << solutionNode.action << ") ";
    }

    SearchAction<GRAPH> action;
};

} // namespace srs

#endif // SOLUTIONNODE_HPP_

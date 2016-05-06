/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SEARCHNODE_HPP_
#define SEARCHNODE_HPP_

namespace srs {

#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/search/SearchAction.hpp>

template<typename GRAPH>
struct SearchNode
{
    typedef SearchPosition<GRAPH> PositionType;

    SearchNode(SearchAction<GRAPH> action, SearchNode<GRAPH>* parent = nullptr) :
        action(action),
        parent(parent)
    {}

    int getTotalCost()
    {
        return action.g + action.h;
    }

    friend ostream& operator<<(ostream& stream, const SearchNode* searchNode)
    {
        stream << hex << reinterpret_cast<long>(searchNode) << dec;
        stream << " (g: " << searchNode->action.g << ", h: " << searchNode->action.h << ") ";
        stream << searchNode->position;

        return stream;
    }

    friend bool operator==(const SearchNode<GRAPH>& lhs, const SearchNode<GRAPH>& rhs)
    {
        return lhs.action.position == rhs.action.position;
    }

    SearchAction<GRAPH> action;
    SearchNode* parent;
};

} // namespace srs

namespace std {

// Hash definition for the SearchNode class
template<typename GRAPH>
struct hash<srs::SearchNode<GRAPH>*>
{
    unsigned long operator()(const srs::SearchNode<GRAPH>* node) const
    {
        return hash<typename srs::SearchPosition<GRAPH>>()(node->action.position);
    }
};

template<typename GRAPH>
struct equal_to<srs::SearchNode<GRAPH>*>
{
    bool operator()(const srs::SearchNode<GRAPH>* lhs, const srs::SearchNode<GRAPH>* rhs) const
    {
        return lhs->action.position == rhs->action.position;
    }
};

} // namespace std

#endif // SEARCHNODE_HPP_

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SEARCHNODE_HPP_
#define SEARCHNODE_HPP_

#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/search/SearchAction.hpp>

namespace srs {

template<typename GRAPH>
struct SearchNode
{
    typedef SearchPosition<GRAPH> PositionType;

    SearchNode(SearchAction<GRAPH>* action, SearchNode<GRAPH>* parent = nullptr) :
        action(action),
        parent(parent)
    {}

    ~SearchNode()
    {
        if (action)
        {
            delete action;
        }
    }

    unsigned int getTotalCost()
    {
        return action->getTotalCost();
    }

    friend ostream& operator<<(ostream& stream, const SearchNode* searchNode)
    {
        return stream << hex << reinterpret_cast<long>(searchNode) << dec << " " <<
            searchNode->action <<
            hex << reinterpret_cast<long>(searchNode->parent) << dec;
    }

    friend bool operator==(const SearchNode<GRAPH>& lhs, const SearchNode<GRAPH>& rhs)
    {
        return lhs.action->position == rhs.action->position;
    }

    SearchAction<GRAPH>* action;
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
        return hash<typename srs::SearchPosition<GRAPH>>()(node->action->position);
    }
};

template<typename GRAPH>
struct equal_to<srs::SearchNode<GRAPH>*>
{
    bool operator()(const srs::SearchNode<GRAPH>* lhs, const srs::SearchNode<GRAPH>* rhs) const
    {
        return (lhs == rhs) || (lhs->action->position == rhs->action->position);
    }
};

} // namespace std

#endif // SEARCHNODE_HPP_

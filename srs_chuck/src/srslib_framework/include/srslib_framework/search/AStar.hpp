/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ASTAR_HPP_
#define ASTAR_HPP_

#include <array>
#include <vector>
#include <limits>
#include <unordered_set>
using namespace std;

#include <srslib_framework/datastructure/MappedPriorityQueue.hpp>

#include <srslib_framework/search/SolutionNode.hpp>
#include <srslib_framework/search/SearchNode.hpp>
#include <srslib_framework/search/SearchPosition.hpp>

namespace srs {

template<typename GRAPH>
class AStar
{
public:
    typedef typename GRAPH::LocationType LocationType;
    typedef SearchNode<GRAPH> SearchNodeType;
    typedef SearchAction<GRAPH> SearchActionType;

    AStar() :
        graph_(nullptr),
        lastNode_(nullptr)
    {}

    AStar(GRAPH* graph) :
        graph_(graph),
        lastNode_(nullptr)
    {}

    ~AStar()
    {}

    void clear()
    {
        // TODO: Use foreach operation in the queue
        SearchNodeType* node = nullptr;
        while (!open_.empty())
        {
            if (open_.pop(node))
            {
                delete node;
            }
        }

        for (auto node : closed_)
        {
            delete node;
        }
        closed_.clear();

        delete lastNode_;
        lastNode_ = nullptr;
    }

    vector<SolutionNode<GRAPH>> getPath()
    {
        vector<SolutionNode<GRAPH>> result;

        SearchNode<GRAPH>* cursor = lastNode_;
        while (cursor)
        {
            SolutionNode<GRAPH> node;
            node.action = cursor->action;

            result.insert(result.begin(), node);
            cursor = cursor->parent;
        }

        return result;
    }

    bool search(SearchPosition<GRAPH> start, SearchPosition<GRAPH> goal)
    {
        clear();
        if (!graph_)
        {
            return false;
        }

        SearchActionType startAction = SearchActionType(SearchActionType::START,
            start, 0, SearchPosition<GRAPH>::heuristic(start, goal));
        SearchNodeType* currentNode = new SearchNodeType(startAction, nullptr);

        SearchActionType goalAction = SearchActionType(SearchActionType::NONE, goal);
        SearchNodeType goalNode = SearchNodeType(goalAction, nullptr);

        open_.push(currentNode->getTotalCost(), currentNode);

        while (!open_.empty())
        {
            open_.pop(currentNode);
            if (*currentNode == goalNode)
            {
                SearchAction<GRAPH> searchAction = SearchAction<GRAPH>::instanceOf(
                    SearchActionType::GOAL,
                    graph_,
                    currentNode);

                lastNode_ = new SearchNodeType(searchAction, currentNode);

                return true;
            }

            closed_.insert(currentNode);

            for (auto action : SearchAction<GRAPH>::ACTIONS)
            {
                SearchAction<GRAPH> searchAction = SearchAction<GRAPH>::instanceOf(
                    action,
                    graph_,
                    currentNode);

                if (searchAction.actionType != SearchAction<GRAPH>::NONE &&
                    searchAction.getTotalCost() < SearchActionType::MAX_COST)
                {
                    SearchNodeType* newNode = new SearchNodeType(searchAction, currentNode);
                    pushSearchNode(newNode);
                }
            }
        }
    }

    void setGraph(GRAPH* const graph)
    {
        graph_ = graph;
    }

private:
    void pushSearchNode(SearchNodeType* node)
    {
        if (!closed_.count(node))
        {
            SearchNodeType* inOpenQueue = open_.find(node);
            if (inOpenQueue)
            {
                if (inOpenQueue->getTotalCost() > node->getTotalCost())
                {
                    open_.erase(inOpenQueue);
                    delete inOpenQueue;

                    open_.push(node->getTotalCost(), node);
                }
                else
                {
                    delete node;
                }
            }
            else
            {
                open_.push(node->getTotalCost(), node);
            }
        }
    }

    unordered_set<SearchNodeType*> closed_;
    MappedPriorityQueue<SearchNodeType*, unsigned int> open_;

    GRAPH* graph_;
    SearchNodeType* lastNode_;
};

} // namespace srs

#endif // ASTAR_HPP_

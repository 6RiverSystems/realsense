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

    AStar(const GRAPH& graph) :
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

    void search(SearchPosition<GRAPH> start, SearchPosition<GRAPH> goal)
    {
        SearchActionType startAction = SearchActionType(SearchActionType::NONE,
            start, 0, SearchPosition<GRAPH>::heuristic(start, goal));
        SearchNodeType* currentNode = new SearchNodeType(startAction, nullptr);

        SearchActionType goalAction = SearchActionType(SearchActionType::NONE, goal);
        SearchNodeType* goalNode = new SearchNodeType(goalAction, nullptr);

        clear();
        open_.push(currentNode->getTotalCost(), currentNode);

        while (!open_.empty())
        {
            open_.pop(currentNode);
            if (*currentNode == *goalNode)
            {
                lastNode_ = currentNode;
                break;
            }

            closed_.insert(currentNode);

            for (auto action : SearchAction<GRAPH>::ACTIONS)
            {
                SearchAction<GRAPH> searchAction = SearchAction<GRAPH>::instanceOf(
                    action,
                    graph_,
                    currentNode);

                if (searchAction.action != SearchAction<GRAPH>::NONE)
                {
                    SearchNodeType* newNode = new SearchNodeType(searchAction, currentNode);

                    pushSearchNode(newNode);
                }
            }
        }
    }

private:
    void pushSearchNode(SearchNodeType* node)
    {
        if (!closed_.count(node))
        {
            if (open_.exists(node))
            {
                SearchNodeType* inOpenQueue = open_.find(node);
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
    MappedPriorityQueue<SearchNodeType*, int> open_;

    GRAPH graph_;
    SearchNodeType* lastNode_;
};

} // namespace srs

#endif // ASTAR_HPP_

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ASTAR_HPP_
#define ASTAR_HPP_

#include <vector>
using namespace std;

#include <srslib_framework/datastructure/PriorityQueue.hpp>

namespace srs {

template<typename GRAPH>
class AStar
{
public:
    typedef typename GRAPH::NodeType NodeType;

    AStar(const GRAPH& graph) :
        graph_(graph)
    {}

    ~AStar();

    vector<NodeType> getPath()
    {
        return path_;
    }

    void search(NodeType start, NodeType goal)
    {
        unordered_map<NodeType, float> costs_;

        queue_.push(start, 0);
        path_.push_back(start);

        while (!queue_.empty())
        {
            auto currentNode = queue_.pop();
            if (currentNode == goal)
            {
                break;
            }

            int currentNodeCost = costs_[currentNode];
            for (auto neighborNode : graph_.neighbors(currentNode))
            {
                int newCost = currentNodeCost; // + graph_.getValue(currentNode, neighborNode);
                costs_[neighborNode] = newCost;

                if (!costs_.count(neighborNode) || newCost < costs_[neighborNode])
                {
                    costs_[neighborNode] = newCost;
                    int newPriority = newCost + GRAPH::getHeuristic(neighborNode, goal);
                    queue_.push_back(neighborNode, newPriority);
                    path_[neighborNode] = currentNode;
                }
            }
        }
    }

private:
    PriorityQueue<NodeType> queue_;
    vector<NodeType> path_;

    GRAPH graph_;
};

} // namespace srs

#endif // ASTAR_HPP_

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

#include <srslib_framework/math/AngleMath.hpp>

#include <srslib_framework/datastructure/MappedPriorityQueue.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/search/SearchNode.hpp>
#include <srslib_framework/search/SearchPosition.hpp>

namespace srs {

// TODO: Solve the problem of the 180 rotation: If 180rot is not allowed,
// A* might be able to produce a sequence of 90 rotations to go around the
// constraint. If we don't want allow that, +/-90 rot should be as expensive
// as a 180rot.
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
            open_.pop(node);
            delete node;
        }

        for (auto node : closed_)
        {
            delete node;
        }
        closed_.clear();

        delete lastNode_;
        lastNode_ = nullptr;
    }

    unsigned int getClosedNodeCount()
    {
        return closed_.size();
    }

    unsigned int getOpenNodeCount()
    {
        return open_.size();
    }

    // TODO: Instead of passing the resolution an object with the map coordinate transformation
    // should be passed
    // TODO: Also this conversion should be done elsewhere.
    // TODO: Change the format of the solution to aggregate similar maneuvers:
    // like consecutive forwards/backwards, and consecutive rotations
    Solution<GRAPH> getSolution(double graphResolution)
    {
        Solution<GRAPH> result;

        SolutionNode<GRAPH> solutionNode;
        Pose<> fromPose;
        Pose<> toPose;

        double fromX = 0;
        double fromY = 0;
        double fromTheta = 0;

        double toX = 0;
        double toY = 0;
        double toTheta = 0;

        SearchNode<GRAPH>* toCursor = lastNode_;
        SearchNode<GRAPH>* fromCursor = lastNode_->parent;

        bool insertNode;

        while (toCursor)
        {
            insertNode = true;

            getWorldCoordinates(graphResolution,
                toCursor->action->position.location.x, toCursor->action->position.location.y,
                toX, toY);

            toTheta = AngleMath::deg2rad<double>(toCursor->action->position.orientation);
            toPose = Pose<>(toX, toY, toTheta);

            if (fromCursor)
            {
                getWorldCoordinates(graphResolution,
                    fromCursor->action->position.location.x, fromCursor->action->position.location.y,
                    fromX, fromY);

                fromTheta = AngleMath::deg2rad<double>(fromCursor->action->position.orientation);
                fromPose = Pose<>(fromX, fromY, fromTheta);
            }

            switch (toCursor->action->actionType)
            {
                case SearchAction<GRAPH>::START:
                    insertNode = false;
                    break;

                case SearchAction<GRAPH>::GOAL:
                    insertNode = false;
                    break;

                case SearchAction<GRAPH>::FORWARD:
                    solutionNode.actionType = SolutionNode<GRAPH>::MOVE;
                    break;

                case SearchAction<GRAPH>::BACKWARD:
                    solutionNode.actionType = SolutionNode<GRAPH>::MOVE;
                    break;

                case SearchAction<GRAPH>::ROTATE_M90:
                    solutionNode.actionType = SolutionNode<GRAPH>::ROTATE;
                    fromTheta = AngleMath::normalizeAngleRad<double>(
                        toTheta + AngleMath::deg2rad<double>(90));
                    fromPose = Pose<>(toX, toY, fromTheta);
                    break;

                case SearchAction<GRAPH>::ROTATE_P90:
                    solutionNode.actionType = SolutionNode<GRAPH>::ROTATE;
                    fromTheta = AngleMath::normalizeAngleRad<double>(
                        toTheta - AngleMath::deg2rad<double>(90));
                    fromPose = Pose<>(toX, toY, fromTheta);
                    break;

                case SearchAction<GRAPH>::ROTATE_180:
                    solutionNode.actionType = SolutionNode<GRAPH>::ROTATE;
                    fromTheta = AngleMath::normalizeAngleRad<double>(
                        toTheta - AngleMath::deg2rad<double>(180));
                    fromPose = Pose<>(toX, toY, fromTheta);
                    break;

                default:
                    // It should never see a NONE
                    throw;
            }

            solutionNode.fromPose = fromPose;
            solutionNode.toPose = toPose;

            if (insertNode)
            {
                result.insert(result.begin(), solutionNode);
            }

            toCursor = toCursor->parent;
            if (fromCursor)
            {
                fromCursor = fromCursor->parent;
            }
        }

        cout << result << endl;

        return result;
    }

    bool search(SearchPosition<GRAPH> start, SearchPosition<GRAPH> goal)
    {
        clear();
        if (!graph_)
        {
            return false;
        }

        // Create the starting node
        SearchActionType* startAction = SearchActionType::instanceOf(
            SearchActionType::START, graph_,
            start,
            SearchPosition<GRAPH>::heuristic(start, goal));
        SearchNodeType* currentNode = new SearchNodeType(startAction, nullptr);

        open_.push(currentNode->getTotalCost(), currentNode);

        // Create the goal node
        SearchActionType* goalAction = SearchActionType::instanceOf(
            SearchActionType::GOAL, graph_,
            goal,
            0);
        SearchNodeType* goalNode = new SearchNodeType(goalAction, nullptr);

        while (!open_.empty())
        {
            // The current node is popped from the priority queue and
            // immediately declared closed
            open_.pop(currentNode);
            closed_.insert(currentNode);

            // If the goal node and the current node are the same, then
            // exit and return the solution. The "==" operator depends on the
            // nature of the SearchNode class
            if (*currentNode == *goalNode)
            {
                // Connect the goal node to the current node
                // and exit
                goalNode->parent = currentNode;
                lastNode_ = goalNode;

                return true;
            }

            for (auto action : SearchActionType::ACTIONS)
            {
                SearchActionType* searchAction = SearchActionType::instanceOf(
                    action,
                    graph_,
                    currentNode);

                if (searchAction)
                {
                    // Do not add any action that leads to the maximum cost
                    if (searchAction->actionType != SearchActionType::NONE &&
                        searchAction->getTotalCost() < SearchActionType::MAX_COST)
                    {
                        SearchNodeType* newNode = new SearchNodeType(searchAction, currentNode);
                        pushSearchNode(newNode);
                    }
                    else
                    {
                        delete searchAction;
                    }
                }
            }
        }

        // If a solution was not found, all the other nodes but the goal node
        // are in the closed list. The goal node can be removed
        delete goalNode;

        return false;
    }

    void setGraph(GRAPH* const graph)
    {
        graph_ = graph;
    }

private:
    // TODO: This should be in the map
    void getWorldCoordinates(double resolution, int c, int r, double& x, double& y)
    {
        x = static_cast<double>(c + 1) * resolution;
        y = static_cast<double>(r + 1) * resolution;

        // The precision is down to 1mm
        x = round(x * 1000) / 1000;
        y = round(y * 1000) / 1000;
    }

    void pushSearchNode(SearchNodeType* node)
    {
        if (!closed_.count(node))
        {
            SearchNodeType* inOpenQueue = open_.find(node);
            if (inOpenQueue)
            {
                // If the total cost of the node is greater than the
                // latest found node, remove the old one and insert
                // the new one
                if (inOpenQueue->getTotalCost() > node->getTotalCost())
                {
                    open_.erase(inOpenQueue);
                    delete inOpenQueue;

                    open_.push(node->getTotalCost(), node);
                }
                else
                {
                    // If the latest node has a total cost that is greater
                    // than what we already have, delete the new node
                    // and do not do anything else
                    delete node;
                }
            }
            else
            {
                // If the node is not in the eopne list
                // add it right away
                open_.push(node->getTotalCost(), node);
            }
        }
        else
        {
            // If the node is already in the closed list, there
            // is no need to add it again. It can be removed
            delete node;
        }
    }

    unordered_set<SearchNodeType*> closed_;

    GRAPH* graph_;

    SearchNodeType* lastNode_;

    MappedPriorityQueue<SearchNodeType*, unsigned int> open_;
};

} // namespace srs

#endif // ASTAR_HPP_

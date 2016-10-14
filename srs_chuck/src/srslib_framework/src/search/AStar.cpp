#include <srslib_framework/search/AStar.hpp>

#include <iostream>
using namespace std;

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStar::clear()
{
    // Pop all the nodes out of the priority queue and
    // release them
    SearchNode* node = nullptr;
    while (!open_.empty())
    {
        open_.pop(node);
        node->release();
    }

    // Release all the nodes in the closed set,
    // except for the start node, which is responsibility
    // of the user
    for (auto node : closed_)
    {
        if (node != startNode_)
        {
            node->release();
        }
    }
    closed_.clear();

    // Clear the last search
    lastNode_ = nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool AStar::search(SearchNode* start, SearchGoal* goal)
{
    // Do not execute any search without complete information
    if (!start || !goal)
    {
        return false;
    }

    startNode_ = start;

    // Make sure that the start node has
    // the specified goal set
    startNode_->setGoal(goal);

    clear();

//    cout << "OPEN ------------------------------------------------------------------------------------------" << endl;
//    cout << open_ << endl << endl;
//    cout << "-----------------------------------------------------------------------------------------------" << endl;
//
//    cout << "CLOSED ----------------------------------------------------------------------------------------" << endl;
//    for (auto node : closed_)
//    {
//        cout << *node << endl;
//    }
//    cout << "-----------------------------------------------------------------------------------------------" << endl;

    // Add the starting node to the open queue
    open_.push(startNode_->getTotalCost(), startNode_);

    vector<SearchNode*> nextSearchNodes;

    SearchNode* currentNode = nullptr;
    while (!open_.empty())
    {
//        cout << "OPEN ------------------------------------------------------------------------------------------" << endl;
//        cout << open_ << endl << endl;
//        cout << "-----------------------------------------------------------------------------------------------" << endl;
//
//        cout << "CLOSED ----------------------------------------------------------------------------------------" << endl;
//        for (auto node : closed_)
//        {
//            cout << *node << endl;
//        }
//        cout << "-----------------------------------------------------------------------------------------------" << endl;

        // The current node is popped from the priority queue and
        // immediately declared closed
        open_.pop(currentNode);
        closed_.insert(currentNode);

//        cout << "===============================================================================================" << endl;
//        cout << *currentNode << endl;
//        cout << "===============================================================================================" << endl;

        // If the current node satisfies its goal, then
        // exit and return the solution
        if (currentNode->goalReached())
        {
            lastNode_ = currentNode;
            return true;
        }

        // Collect all the valid next states from
        // from the current node
        currentNode->getNeighbors(nextSearchNodes);

        // Try to push the neighbors in the open queue
        pushNodes(nextSearchNodes);
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStar::pushNodes(vector<SearchNode*>& nodes)
{
//    cout << "NEXT ------------------------------------------------------------------------------------------" << endl;
//    for (auto node : nodes)
//    {
//        cout << *node << endl;
//    }
//    cout << "-----------------------------------------------------------------------------------------------" << endl;
//
//    cout << "OPEN ------------------------------------------------------------------------------------------" << endl;
//    cout << open_ << endl << endl;
//    cout << "-----------------------------------------------------------------------------------------------" << endl;
//
//    cout << "CLOSED ----------------------------------------------------------------------------------------" << endl;
//    for (auto node : closed_)
//    {
//        cout << *node << endl;
//    }
//    cout << "-----------------------------------------------------------------------------------------------" << endl;

    for (auto node : nodes)
    {
//        cout << "Evaluating: " << *node;

        if (!closed_.count(node))
        {
            SearchNode* inOpenQueue = open_.find(node);
            if (inOpenQueue)
            {
                // If the total cost of the node is greater than the
                // latest found node, remove the old one and insert
                // the new one
                if (inOpenQueue->getTotalCost() > node->getTotalCost())
                {
                    open_.erase(inOpenQueue);
                    inOpenQueue->release();

                    open_.push(node->getTotalCost(), node);
                    // cout << endl << "Better solution: pushed" << endl;
                }
                else
                {
                    // If the latest node has a total cost that is greater
                    // than what we already have, delete the new node
                    // and do not do anything else
                    node->release();
                    // cout << endl << "Worse solution: pruned (freed)" << endl;
                }
            }
            else
            {
                // If the node is not in the open list
                // add it right away
                open_.push(node->getTotalCost(), node);
                // cout << endl << "Inserted" << endl;
            }
        }
        else
        {
            // If the node is already in the closed list, there
            // is no need to add it again. It can be removed
            node->release();
            // cout << endl << "In the closed list: pruned (freed)" << endl;
        }
    }

    nodes.clear();
    // cout << "DONE ------------------------------------------------------------------------------------------" << endl << endl;
}

} // namespace srs

#include <srslib_framework/search/AStar.hpp>

#define DEBUG_ASTAR 0
#define YIELD_ENABLED 0

#include <iostream>

#ifdef YIELD_ENABLED
    #include <thread>
#endif

using namespace std;

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
AStar::AStar() :
    lastNode_(nullptr),
    startNode_(nullptr)
{
    closedSet_.reserve(CLOSED_HASH_RESERVE);

    clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
AStar::~AStar()
{
    clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStar::clear()
{
    // Pop all the nodes out of the priority queue and
    // release them
    SearchNode* node = nullptr;
    while (!openQueue_.empty())
    {
        openQueue_.pop(node);
        if (node != startNode_)
        {
            node->release();
        }
    }

    // Release all the nodes in the closed set,
    // except for the start node, which is responsibility
    // of the user
    for (auto node : closedSet_)
    {
        if (node != startNode_)
        {
            node->release();
        }
    }
    closedSet_.clear();

    // Clear the last search and the yield counter
    lastNode_ = nullptr;

    // Reset the stats counters
    counterYield_ = 0;
    counterInserted_ = 0;
    counterFoundInClosed_ = 0;
    counterReplaced_ = 0;
    counterPruned_ = 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStar::getPlan(Plan& plan)
{
    plan.clear();

    plan.setValid(lastNode_ != nullptr);
    plan.setClosedNodesCount(closedSet_.size());
    plan.setOpenNodesCount(openQueue_.size());
    plan.setTotalCost(lastNode_->getLocalCost());

    // Explore the tree backward to
    // reconstruct the solution
    SearchNode* node = lastNode_;
    while (node)
    {
        plan.push_front(node);
        node = node->getParent();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool AStar::search(SearchNode* start, SearchGoal* goal, ConfigParameters parameters)
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

    // Add the starting node to the open queue
    openQueue_.push(startNode_->getTotalCost(), startNode_);

    vector<SearchNode*> nextSearchNodes;

    SearchNode* currentNode = nullptr;
    while (!openQueue_.empty())
    {
        #if DEBUG_ASTAR
            cout << "OPEN ------------------------------------------------------------------------------------------" << endl;
            cout << openQueue_ << endl << endl;
            cout << "-----------------------------------------------------------------------------------------------" << endl;

            cout << "CLOSED ----------------------------------------------------------------------------------------" << endl;
            for (auto node : closedSet_)
            {
                cout << *node << endl;
            }
            cout << "-----------------------------------------------------------------------------------------------" << endl;
        #endif

        // The current node is popped from the priority queue and
        // immediately declared closed
        openQueue_.pop(currentNode);
        closedSet_.insert(currentNode);

        #if DEBUG_ASTAR
            cout << "===============================================================================================" << endl;
            cout << *currentNode << endl;
            cout << "===============================================================================================" << endl;
        #endif

        // If the current node satisfies its goal, then
        // exit and return the solution
        if (currentNode->goalReached())
        {
            lastNode_ = currentNode;
            return true;
        }

        // Collect all the valid next states from
        // from the current node
        currentNode->getExploredNodes(nextSearchNodes);

        #if DEBUG_ASTAR
            cout << "NEXT ------------------------------------------------------------------------------------------" << endl;
            for (auto node : nextSearchNodes)
            {
                cout << *node << endl;
            }
            cout << "-----------------------------------------------------------------------------------------------" << endl;
        #endif

        // Try to push the neighbors in the open queue
        pushNodes(nextSearchNodes);

        // If the yield property has been set
        #ifdef YIELD_ENABLED
            counterYield_++;
            if (parameters.useYield && counterYield_ > parameters.yieldFrequency)
            {
                counterYield_ = 0;
                this_thread::yield();
            }
        #endif
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void AStar::pushNodes(vector<SearchNode*>& nodes)
{
    #if DEBUG_ASTAR
        cout << "OPEN ------------------------------------------------------------------------------------------" << endl;
        cout << openQueue_ << endl << endl;
        cout << "-----------------------------------------------------------------------------------------------" << endl;

        cout << "CLOSED ----------------------------------------------------------------------------------------" << endl;
        for (auto node : closedSet_)
        {
            cout << *node << endl;
        }
        cout << "-----------------------------------------------------------------------------------------------" << endl;
    #endif

    for (int i = 0; i < nodes.size(); i++)
    {
        SearchNode* node = nodes[i];

        #if DEBUG_ASTAR
            cout << "Evaluating: " << *node;
        #endif

        if (closedSet_.find(node) == closedSet_.end())
        {
            SearchNode* inOpenQueue = openQueue_.find(node);
            if (inOpenQueue)
            {
                // If the total cost of the node is greater than the
                // latest found node, remove the old one and insert
                // the new one
                if (inOpenQueue->getTotalCost() > node->getTotalCost())
                {
                    counterReplaced_++;

                    openQueue_.erase(inOpenQueue);
                    inOpenQueue->release();

                    openQueue_.push(node->getTotalCost(), node);

                    #if DEBUG_ASTAR
                        cout << endl << "Better solution: pushed" << endl;
                    #endif
                }
                else
                {
                    counterPruned_++;

                    // If the latest node has a total cost that is greater
                    // than what we already have, delete the new node
                    // and do not do anything else
                    node->release();

                    #if DEBUG_ASTAR
                        cout << endl << "Worse solution: pruned (freed)" << endl;
                    #endif
                }
            }
            else
            {
                counterInserted_++;

                // If the node is not in the open list
                // add it right away
                openQueue_.push(node->getTotalCost(), node);

                #if DEBUG_ASTAR
                    cout << endl << "Inserted" << endl;
                #endif
            }
        }
        else
        {
            counterFoundInClosed_++;

            // If the node is already in the closed list, there
            // is no need to add it again. It can be removed
            node->release();

            #if DEBUG_ASTAR
                cout << endl << "In the closed list: pruned (freed)" << endl;
            #endif
        }
    }

    nodes.clear();

    #if DEBUG_ASTAR
        cout << "DONE ------------------------------------------------------------------------------------------" << endl << endl;
    #endif
}

} // namespace srs

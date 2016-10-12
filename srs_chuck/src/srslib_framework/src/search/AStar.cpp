namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename HASH, typename EQUAL_TO>
void AStar<HASH, EQUAL_TO>::clear()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename HASH, typename EQUAL_TO>
bool AStar<HASH, EQUAL_TO>::search(ISearchNode* startNode)
{
    clear();

    // Add the starting node to the open queue
    open_.push(startNode->getTotalCost(), startNode);

    vector<ISearchNode*> neighbors;

    ISearchNode* currentNode = nullptr;
    while (!open_.empty())
    {
        // The current node is popped from the priority queue and
        // immediately declared closed
        open_.pop(currentNode);
        closed_.insert(currentNode);

        cout << open_ << endl;
        cout << currentNode << endl;

        // If the goal node and the current node are the same, then
        // exit and return the solution. The "==" operator depends on the
        // nature of the ISearchNode class
        if (currentNode->reachedGoal())
        {
            lastNode_ = currentNode;
            return true;
        }

        currentNode->getNeighbors(neighbors);
        pushNodes(neighbors);
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename HASH, typename EQUAL_TO>
void AStar<HASH, EQUAL_TO>::pushNodes(vector<ISearchNode*>& nodes)
{
    for (auto node : nodes)
    {
        if (!closed_.count(node))
        {
            ISearchNode* inOpenQueue = open_.find(node);
            if (inOpenQueue)
            {
                // If the total cost of the node is greater than the
                // latest found node, remove the old one and insert
                // the new one
                if (inOpenQueue->getTotalCost() > node->getTotalCost())
                {
                    open_.erase(inOpenQueue);
                    inOpenQueue->freeNode();

                    open_.push(node->getTotalCost(), node);
                }
                else
                {
                    // If the latest node has a total cost that is greater
                    // than what we already have, delete the new node
                    // and do not do anything else
                    node->freeNode();
                }
            }
            else
            {
                // If the node is not in the open list
                // add it right away
                open_.push(node->getTotalCost(), node);
            }
        }
        else
        {
            // If the node is already in the closed list, there
            // is no need to add it again. It can be removed
            node->freeNode();
        }
    }
}

} // namespace srs

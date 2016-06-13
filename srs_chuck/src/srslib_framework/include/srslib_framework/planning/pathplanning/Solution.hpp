/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SOLUTION_HPP_
#define SOLUTION_HPP_

#include <iomanip>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

// TODO: The Solution node should be a solution only for the Grid2d graph.
// Each graph type should have its own solution node.
template<typename GRAPH>
struct SolutionNode
{
    enum ActionEnum {NONE, START, GOAL, MOVE, ROTATE};

    SolutionNode() :
        actionType(NONE),
        fromPose(Pose<>()),
        toPose(Pose<>()),
        cost(0.0)
    {}

    SolutionNode(ActionEnum actionType, Pose<> fromPose, Pose<> toPose, double cost = 0.0) :
        actionType(actionType),
        fromPose(fromPose),
        toPose(toPose),
        cost(cost)
    {}

    friend ostream& operator<<(ostream& stream, const SolutionNode& solutionNode)
    {
        return stream << " (" << ENUM_NAMES[solutionNode.actionType] <<
            ", " << solutionNode.fromPose << " -> " <<
                    solutionNode.toPose << ", " << solutionNode.cost << ")";
    }

    ActionEnum actionType;

    double cost;

    Pose<> fromPose;

    Pose<> toPose;

private:
    static unordered_map<int, string> ENUM_NAMES;
};

template<typename GRAPH>
class Solution : public vector<SolutionNode<GRAPH>>
{
public:
    typedef SolutionNode<GRAPH> NodeType;

    Solution()
    {}

    NodeType afterStart()
    {
        return *(vector<NodeType>::begin() + 1);
    }

    NodeType beforeGoal()
    {
        return *(vector<NodeType>::end() - 2);
    }

    friend ostream& operator<<(ostream& stream, const Solution& solution)
    {
        int counter = 0;

        stream << "Solution {" << endl;
        for (auto node : solution)
        {
            stream << setw(4) << counter++ << ": " << node << endl;
        }

        return stream << "}";
    }

    void split(unsigned int position, Solution<GRAPH>& first, Solution<GRAPH>& second)
    {
        first.clear();
        second.clear();

        for (unsigned int i = 0; i <= position; i++)
        {
            first.push_back(this->at(i));
        }

        for (unsigned int i = position + 1; i < this->size(); i++)
        {
            second.push_back(this->at(i));
        }
    }
};

template<typename GRAPH>
unordered_map<int, string> SolutionNode<GRAPH>::ENUM_NAMES = {
    {SolutionNode<GRAPH>::NONE, "NONE"},
    {SolutionNode<GRAPH>::START, "START"},
    {SolutionNode<GRAPH>::GOAL, "GOAL"},
    {SolutionNode<GRAPH>::MOVE, "MOVE"},
    {SolutionNode<GRAPH>::ROTATE, "ROTATE"},
};

} // namespace srs

#endif // SOLUTION_HPP_

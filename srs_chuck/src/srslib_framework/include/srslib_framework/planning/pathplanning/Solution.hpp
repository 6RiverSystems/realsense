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
    enum ActionEnum {NONE, START, GOAL, FORWARD, BACKWARD, ROTATE_M90, ROTATE_P90, ROTATE_180};

    SolutionNode() :
        actionType(NONE),
        pose(),
        cost(0.0)
    {}

    SolutionNode(ActionEnum actionType,
            Pose<> pose,
            double cost = 0.0) :
        actionType(actionType),
        pose(pose),
        cost(cost)
    {}

    friend ostream& operator<<(ostream& stream, const SolutionNode& solutionNode)
    {
        return stream << " (" << ENUM_NAMES[solutionNode.actionType] <<
            ", " << solutionNode.pose << ", " << solutionNode.cost << ")";
    }

    ActionEnum actionType;

    double cost;

    Pose<> pose;

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
};

template<typename GRAPH>
unordered_map<int, string> SolutionNode<GRAPH>::ENUM_NAMES = {
    {SolutionNode<GRAPH>::NONE, "NONE"},
    {SolutionNode<GRAPH>::START, "START"},
    {SolutionNode<GRAPH>::GOAL, "GOAL"},
    {SolutionNode<GRAPH>::FORWARD, "FORWARD"},
    {SolutionNode<GRAPH>::BACKWARD, "BACKWARD"},
    {SolutionNode<GRAPH>::ROTATE_M90, "ROTATE_M90"},
    {SolutionNode<GRAPH>::ROTATE_P90, "ROTATE_P90"},
    {SolutionNode<GRAPH>::ROTATE_180, "ROTATE_180"}
};

} // namespace srs

#endif // SOLUTION_HPP_

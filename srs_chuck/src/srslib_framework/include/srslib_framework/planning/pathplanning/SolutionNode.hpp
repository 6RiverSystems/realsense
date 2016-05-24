/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef SOLUTIONNODE_HPP_
#define SOLUTIONNODE_HPP_

namespace srs {

template<typename GRAPH>
struct SolutionNode
{
    typedef typename GRAPH::LocationType LocationType;

    enum ActionEnum {NONE, START, GOAL, FORWARD, BACKWARD, ROTATE_M90, ROTATE_P90, ROTATE_180};

    SolutionNode() :
        actionType(NONE),
        location(),
        orientation(0),
        cost(0)
    {}

    SolutionNode(ActionEnum actionType,
            LocationType location,
            double orientation,
            unsigned int cost) :
        actionType(actionType),
        location(location),
        orientation(orientation),
        cost(cost)
    {}

    friend ostream& operator<<(ostream& stream, const SolutionNode& solutionNode)
    {
        return stream << " (" << ENUM_NAMES[solutionNode.actionType] <<
            ", " << solutionNode.location <<
            ", " << solutionNode.orientation <<
            ", " << solutionNode.cost << ")";
    }

    ActionEnum actionType;

    LocationType location;
    double orientation;

    unsigned int cost;

private:
    static unordered_map<int, string> ENUM_NAMES;
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

#endif // SOLUTIONNODE_HPP_

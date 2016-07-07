/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef GRIDSOLUTIONITEM_HPP_
#define GRIDSOLUTIONITEM_HPP_

#include <iomanip>
#include <unordered_map>
using namespace std;

#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

struct GridSolutionItem
{
    enum ActionEnum {
        NONE,
        MOVE,
        ROTATE};

    GridSolutionItem() :
        actionType(NONE),
        fromPose(Pose<>::ZERO),
        toPose(Pose<>::ZERO),
        cost(0.0)
    {}

    GridSolutionItem(ActionEnum actionType, Pose<> fromPose, Pose<> toPose, double cost = 0.0) :
        actionType(actionType),
        fromPose(fromPose),
        toPose(toPose),
        cost(cost)
    {}

    friend ostream& operator<<(ostream& stream, const GridSolutionItem& solutionItem)
    {
        return stream << " (" << ENUM_NAMES[solutionItem.actionType] <<
            ", " << solutionItem.fromPose << " -> " <<
            solutionItem.toPose << ", " << solutionItem.cost << ")";
    }

    ActionEnum actionType;

    double cost;

    Pose<> fromPose;

    Pose<> toPose;

private:
    static unordered_map<int, string> ENUM_NAMES;
};

} // namespace srs

#endif // GRIDSOLUTIONITEM_HPP_

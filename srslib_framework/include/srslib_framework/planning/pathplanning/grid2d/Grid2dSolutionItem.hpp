/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <iomanip>
#include <unordered_map>
using namespace std;

#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

struct Grid2dSolutionItem
{
    enum ActionEnum {
        NONE,
        MOVE,
        ROTATE};

    Grid2dSolutionItem() :
        actionType(NONE),
        fromPose(Pose<>::ZERO),
        toPose(Pose<>::ZERO),
        cost(0)
    {}

    Grid2dSolutionItem(ActionEnum actionType, Pose<> fromPose, Pose<> toPose, int cost = 0) :
        actionType(actionType),
        fromPose(fromPose),
        toPose(toPose),
        cost(cost)
    {}

    friend ostream& operator<<(ostream& stream, const Grid2dSolutionItem& solutionItem)
    {
        return stream << " (" <<
            ENUM_NAMES[solutionItem.actionType] <<
            ", " << solutionItem.fromPose <<
            " -> " << solutionItem.toPose <<
            ", " << solutionItem.cost <<
            ")";
    }

    friend bool operator==(const Grid2dSolutionItem& lhs, const Grid2dSolutionItem& rhs)
    {
        if (&lhs == &rhs)
        {
            return true;
        }

        return lhs.actionType == rhs.actionType &&
            PoseMath::equal(lhs.fromPose, rhs.fromPose) &&
            PoseMath::equal(lhs.toPose, rhs.toPose) &&
            lhs.cost == rhs.cost;
    }

    ActionEnum actionType;

    Pose<> fromPose;

    Pose<> toPose;

    int cost;

private:
    static unordered_map<int, string> ENUM_NAMES;
};

} // namespace srs

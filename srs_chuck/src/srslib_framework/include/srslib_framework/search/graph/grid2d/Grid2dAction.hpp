/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <limits>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/SearchGoal.hpp>

namespace srs {

class Grid2dNode;

class Grid2dAction
{
public:
    enum ActionEnum {
        BACKWARD = 0,
        FORWARD,
        NONE,
        ROTATE_N90, ROTATE_P90, ROTATE_180,
        START
    };

    typedef pair<Grid2dPosition, Grid2d::BaseType> ActionResultType;

    static array<ActionEnum, 3> ALLOWED_ACTIONS;

    static bool execute(Grid2d* graph, Grid2dNode* fromNode, ActionEnum action,
        ActionResultType& result);

    friend ostream& operator<<(ostream& stream, const ActionEnum& action)
    {
        return stream << ENUM_NAMES[action];
    }

private:
    static constexpr int COMMAND_COSTS_LENGTH = 8;

    static unordered_map<int, string> ENUM_NAMES;
    static const int COMMAND_COSTS[ActionEnum::START + 1];

    static bool addBackward(Grid2d* graph, Grid2dNode* fromNode,
        ActionResultType& result);
    static bool addForward(Grid2d* graph, Grid2dNode* fromNode,
        ActionResultType& result);
    static bool addRotation(Grid2d* graph, Grid2dNode* fromNode,
        ActionEnum action, int angle,
        ActionResultType& result);
};

} // namespace srs

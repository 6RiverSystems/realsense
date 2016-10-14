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
#include <srslib_framework/search/ISearchGoal.hpp>

namespace srs {

class Grid2dNode;

class Grid2dAction
{
public:
    enum ActionEnum {
        BACKWARD = 0,
        FORWARD,
        GOAL,
        NONE,
        ROTATE_N90, ROTATE_P90, ROTATE_180,
        START
    };

    typedef pair<Grid2dPosition, int> ActionResultType;

    static array<ActionEnum, 3> ALLOWED_ACTIONS;

    static ActionResultType execute(Grid2d* graph, Grid2dNode* fromNode, ActionEnum action);

    friend ostream& operator<<(ostream& stream, const ActionEnum& action)
    {
        return stream << ENUM_NAMES[action];
    }

private:
    static constexpr int COMMAND_COSTS_LENGTH = 8;

    static unordered_map<int, string> ENUM_NAMES;
    static const int COMMAND_COSTS[ActionEnum::START + 1];

    static ActionResultType addBackward(Grid2d* graph, Grid2dNode* fromNode);
    static ActionResultType addForward(Grid2d* graph, Grid2dNode* fromNode);
    static ActionResultType addRotation(Grid2d* graph, Grid2dNode* fromNode,
        ActionEnum action, int angle);

};

} // namespace srs

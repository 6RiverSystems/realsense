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

#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/search/SearchGoal.hpp>

namespace srs {

class MapStackNode;

class MapStackAction
{
public:
    enum ActionEnum {
        BACKWARD = 0,
        FORWARD,
        NONE,
        ROTATE_N90, ROTATE_P90, ROTATE_180,
        START
    };

    typedef pair<Grid2d::Position, Grid2d::BaseType> ActionResultType;

    static array<ActionEnum, 3> ALLOWED_ACTIONS;

    static bool execute(MapStack* stack, MapStackNode* fromNode, ActionEnum action,
        ActionResultType& result);

    static bool isCostAvailable(int& cost);

    friend ostream& operator<<(ostream& stream, const ActionEnum& action)
    {
        return stream << ENUM_NAMES[action];
    }

private:
    static constexpr int COMMAND_COSTS_LENGTH = 8;

    static unordered_map<int, string> ENUM_NAMES;
    static const int COMMAND_COSTS[ActionEnum::START + 1];

    static bool addBackward(MapStack* stack, MapStackNode* fromNode,
        ActionResultType& result);
    static bool addForward(MapStack* stack, MapStackNode* fromNode,
        ActionResultType& result);
    static bool addRotation(MapStack* stack, MapStackNode* fromNode,
        ActionEnum action, int angle,
        ActionResultType& result);
};

} // namespace srs

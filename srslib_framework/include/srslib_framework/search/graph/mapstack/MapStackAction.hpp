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

namespace srs {

struct MapStackNode;
struct MapStackSearchParameters;

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

    static MapStackNode* exploreBackward(MapStack* stack, const MapStackSearchParameters& searchParameters, MapStackNode* fromNode);
    static MapStackNode* exploreForward(MapStack* stack, const MapStackSearchParameters& searchParameters, MapStackNode* fromNode);
    static MapStackNode* exploreRotation(MapStack* stack, const MapStackSearchParameters& searchParameters, MapStackNode* fromNode,
        ActionEnum action, int angle);

    friend ostream& operator<<(ostream& stream, const ActionEnum& action)
    {
        return stream << ENUM_NAMES[action];
    }

private:
    /*
     * This parameter specifies when a new MapStackAction is pruned because its
     * cost is too high to be considered even a possibility
     */
    static constexpr int PRUNING_THRESHOLD = 254;

    static constexpr int COMMAND_COSTS_LENGTH = 8;

    static unordered_map<int, string> ENUM_NAMES;

    static const int COMMAND_COSTS[ActionEnum::START + 1];
};

} // namespace srs

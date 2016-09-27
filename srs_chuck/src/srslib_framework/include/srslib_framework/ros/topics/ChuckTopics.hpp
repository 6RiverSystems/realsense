/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

namespace ChuckTopics {

namespace external {

    static const string RESPONSE_ARRIVED = "/response/arrived";

} // namespace external

namespace internal {

    static const string GOAL_TO_NAVIGATION = "/move_base_simple/goal";

    static const string GOAL_TARGET_AREA = "/internal/state/goal/target_area";

    static const string MAP_METADATA = "/internal/state/map/metadata";
    static const string MAP_OCCUPANCY = "/internal/state/map/occupancy";
    static const string MAP_LOGICAL = "/internal/state/map/complete";

} // namespace internal

namespace service {

    static const string MAP_OCCUPANCY = "static_map";

} // namespace service

} // namespace ChuckTopics

} // namespace srs

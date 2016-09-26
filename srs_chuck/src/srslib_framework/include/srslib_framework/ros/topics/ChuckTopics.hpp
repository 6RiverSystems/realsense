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
    static const string TARGET_AREA = "/internal/state/goal/target_area";
} // namespace internal

} // namespace ChuckTopics

} // namespace srs

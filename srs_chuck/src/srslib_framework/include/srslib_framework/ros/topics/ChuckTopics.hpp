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

namespace debug {

    static const string ACC_ODOMETRY = "/internal/state/robot/acc_odometry";

} // namespace debug

namespace internal {

    static const string GOAL_ARRIVED = "/internal/state/goal/arrived";
    static const string GOAL_GOAL = "/internal/state/goal/goal";
    static const string GOAL_PATH = "/internal/state/goal/path";
    static const string GOAL_TARGET_AREA = "/internal/state/goal/target_area";

    static const string GOTO_GOAL = "/move_base_simple/goal";

    static const string INITIAL_POSE = "/internal/command/initial_pose";

    static const string MAP_LOGICAL = "/internal/state/map/complete";
    static const string MAP_ROS_OCCUPANCY = "/internal/state/map/ros_occupancy";
    static const string MAP_ROS_METADATA = "/internal/state/map/ros_metadata";
    static const string MAP_STACK = "/internal/state/map/stack";

    static const string ROBOT_POSE = "/internal/state/robot/pose";

} // namespace internal

namespace sensor {

    static const string JOYPAD_STATE = "/internal/sensors/joypad/state";

    static const string JOYSTICK_RAW = "/joy";
}

namespace service {

    static const string GET_MAP_OCCUPANCY = "static_map";

} // namespace service

} // namespace ChuckTopics

} // namespace srs
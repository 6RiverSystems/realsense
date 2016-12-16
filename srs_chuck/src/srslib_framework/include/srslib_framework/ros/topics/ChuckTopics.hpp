/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>

using namespace std;

namespace srs {

namespace ChuckTopics {

namespace external {

    static const string RESPONSE_ARRIVED = "/response/arrived";

} // namespace external

namespace debug {

    static const string ACC_ODOMETRY = "/internal/state/robot/acc_odometry";

} // namespace debug

namespace driver {

	static const string BRAINSTEM_CMD_SHUTDOWN = "/internal/drivers/brainstem/cmd/shutdown";
    static const string BRAINSTEM_CMD_SOUND = "/internal/drivers/brainstem/cmd/sound";
    static const string BRAINSTEM_CMD_UPDATE_LIGHTS = "/internal/drivers/brainstem/cmd/update_ui";
    static const string BRAINSTEM_CMD_PING = "/internal/drivers/brainstem/cmd/ping";
    static const string BRAINSTEM_CMD_SET_MOTION_STATE = "/internal/drivers/brainstem/cmd/set_motion_state";
    static const string BRAINSTEM_STATE_CONNECTED = "/internal/drivers/brainstem/state/connected";
    static const string BRAINSTEM_BUTTON_PRESSED = "/internal/drivers/brainstem/button_pressed";
    static const string BRAINSTEM_ODOMETRY_RPM = "/internal/sensors/odometry/rpm/raw";
    static const string BRAINSTEM_ODOMETRY_RPM_CMD = "/internal/sensors/odometry/rpm/cmd";
    static const string ODOMETRY_CMD_VELOCITY = "/internal/sensors/odometry/velocity/cmd";
    static const string BRAINSTEM_STATE_POWER = "/info/power";
    static const string BRAINSTEM_HARDWARE_INFO = "/info/hardware";
    static const string BRAINSTEM_OPERATIONAL_STATE = "/info/operational_state";
} // namespace debug

namespace internal {

    static const string DANGER_ZONE = "/internal/state/danger_zone/zone";
    static const string DEBUG_DANGER_ZONE = "/internal/state/danger_zone/debug/zone";

    static const string DEBUG_DANGER_ZONE_LASER_SCAN = "/internal/state/danger_zone/debug/scan";

    static const string GOAL_ARRIVED = "/internal/state/goal/arrived";
    static const string GOAL_GOAL = "/internal/state/goal/goal";
    static const string GOAL_PATH = "/internal/state/goal/path";
    static const string GOAL_TARGET_AREA = "/internal/state/goal/target_area";

    static const string GOTO_GOAL = "/move_base_simple/goal";

    static const string MAP_STACK = "/internal/state/map/stack";
    static const string MAP_ROS_AMCL_OCCUPANCY = "/internal/state/map/ros_amcl_occupancy";
    static const string MAP_ROS_LOGICAL = "/internal/state/map/ros_logical";
    static const string MAP_ROS_METADATA = "/internal/state/map/ros_metadata";
    static const string MAP_ROS_OCCUPANCY = "/internal/state/map/ros_occupancy";
    static const string MAP_ROS_PATH = "/internal/state/map/ros_path";
    static const string MAP_ROS_WEIGHTS_EAST = "/internal/state/map/weights_east";
    static const string MAP_ROS_WEIGHTS_NORTH = "/internal/state/map/weights_north";
    static const string MAP_ROS_WEIGHTS_SOUTH = "/internal/state/map/weights_south";
    static const string MAP_ROS_WEIGHTS_WEST = "/internal/state/map/weights_west";

    static const string ODOMETRY_VELOCITY_ESTIMATE = "/internal/sensors/odometry/velocity/estimate";

    static const string ODOMETRY_INITIAL_POSE = "/request/odometry/initial_pose";
    static const string SWCMD_INITIAL_POSE = "/internal/sw_cmd/initial_pose";

} // namespace internal

namespace monitoring {

    static const string TIMING_DATA = "/monitoring/timing_data";

} // namespace monitoring

namespace sensor {
    static const string FILTERED_LIDAR = "/internal/sensors/lidar/scan/filtered";

    static const string JOYPAD_STATE = "/internal/sensors/joypad/state";

    static const string JOYSTICK_RAW = "/joy";

    static const string ODOMETRY_POSE = "/internal/sensors/odometry/pose";
}

namespace service {

    static const string GET_MAP_OCCUPANCY = "static_map";

} // namespace service


namespace tools {

    static const string RACE = "/internal/command/race";

} // namespace node


} // namespace ChuckTopics

} // namespace srs

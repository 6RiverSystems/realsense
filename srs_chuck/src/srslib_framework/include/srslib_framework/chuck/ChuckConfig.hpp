/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

namespace ChuckConfig {

namespace Entities {

    static const string LOCAL_PLANNER = "/move_base/DWAPlannerROS";

} // Entities

namespace Transforms {

    static const string BASE_FOOTPRINT = "base_footprint";

    static const string LIDAR = "sick_lidar";

    static const string MAP = "map";

    static const string ODOMETRY = "odom";

} // namespace Transforms

namespace Parameters {

    static const string FRAME_ID = "frame_id";

    static const string LP_MODE = "mode";
    static const int LP_MODE_DEFAULT = 0;
    static const int LP_MODE_QUEUE = 1;
    static const int LP_MODE_STAY_ON_PATH = 2;

    static const string MAP_STACK = "map_stack";
    static const string MAX_VELOCITY = "max_vel_x";

} // namespace Parameters

} // namespace ChuckConfig

} // namespace srs
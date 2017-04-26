/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>

namespace srs {
    geometry_msgs::PoseStamped shiftGoalToMinima(geometry_msgs::PoseStamped goal_pose,
            const costmap_2d::Costmap2D& costmap, double max_shift);
}


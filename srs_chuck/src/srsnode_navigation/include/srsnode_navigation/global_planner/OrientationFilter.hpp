/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <nav_msgs/Path.h>

namespace srs {

enum OrientationMode { NONE, FORWARD, INTERPOLATE, FORWARDTHENINTERPOLATE };

class OrientationFilter
{
public:
    OrientationFilter() : omode_(NONE) {}
    ~OrientationFilter();

    void processPath(const geometry_msgs::PoseStamped& start,
                             std::vector<geometry_msgs::PoseStamped>& path);

    void pointToNext(std::vector<geometry_msgs::PoseStamped>& path, int index);
    void interpolate(std::vector<geometry_msgs::PoseStamped>& path,
                     int start_index, int end_index);

    void setMode(OrientationMode new_mode){ omode_ = new_mode; }
    void setMode(int new_mode){ setMode((OrientationMode) new_mode); }
protected:
    OrientationMode omode_;
};

}

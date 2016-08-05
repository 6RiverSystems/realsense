/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef POSEADAPTER_HPP_
#define POSEADAPTER_HPP_

#include <vector>
using namespace std;

#include <srslib_framework/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/localization/Map.hpp>

#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

struct PoseAdapter
{
    static void pose2Map(Pose<> pose, Map* map, Grid2d::LocationType& location, int& orientation)
    {
        int r = 0;
        int c = 0;
        map->getMapCoordinates(pose.x, pose.y, c, r);

        location = Grid2d::LocationType(c, r);
        orientation = AngleMath::normalizeRad2deg90(pose.theta);
    }
};

} // namespace srs

#endif // POSEADAPTER_HPP_
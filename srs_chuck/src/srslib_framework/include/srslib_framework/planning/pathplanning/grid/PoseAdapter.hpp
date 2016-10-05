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

#include <srslib_framework/localization/map/BaseMap.hpp>

#include <srslib_framework/robotics/Pose.hpp>

namespace srs {

struct PoseAdapter
{
    static void pose2Map(Pose<> pose, BaseMap* map, Grid2d::LocationType& location, int& orientation)
    {
        unsigned int r = 0;
        unsigned int c = 0;
        map->transformM2Cells(pose, c, r);

        location = Grid2d::LocationType(c, r);
        orientation = static_cast<int>(AngleMath::normalizeRad2Deg90(pose.theta));
    }
};

} // namespace srs

#endif // POSEADAPTER_HPP_

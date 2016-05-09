/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <srslib_framework/search/SolutionNode.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs {

struct Trajectory
{
    static void solution2Pose(vector<SolutionNode>& solution, vector<Pose>& poses)
    {
        poses = vector<Pose>();
    }

    static void pose2velocity(vector<Pose>& poses, vector<Velocity>& velocities)
    {
        velocities = vector<Velocity>();
    }
};

} // namespace srs

#endif // TRAJECTORY_HPP_

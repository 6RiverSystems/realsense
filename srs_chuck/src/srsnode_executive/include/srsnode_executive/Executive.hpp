/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef HIGHLEVELBEHAVIOR_HPP_
#define HIGHLEVELBEHAVIOR_HPP_

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/AStar.hpp>

#include <srsnode_executive/tap/RosTapGoal.hpp>

namespace srs {

class Executive
{
public:
    Executive();

    ~Executive()
    {}

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 5;
    constexpr static int GRID_SIZE = 60;

    void planToGoal(Pose<> goal);
    void publishPlan();

    RosTapGoal tapGoal_;
    ros::NodeHandle rosNodeHandle_;
    Grid2d grid_;
    AStar<Grid2d> algorithm_;

    ros::Publisher pubPlan_;
    int inc_;
};

} // namespace srs

#endif  // HIGHLEVELBEHAVIOR_HPP_

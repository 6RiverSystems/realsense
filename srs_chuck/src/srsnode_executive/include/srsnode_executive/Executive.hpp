/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef EXECUTIVE_HPP_
#define EXECUTIVE_HPP_

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/AStar.hpp>

#include <srsnode_executive/tap/RosTapCmd_Goal.hpp>
#include <srsnode_executive/tap/RosTapCmd_YouAreHere.hpp>

namespace srs {

class Executive
{
public:
    Executive(string nodeName);

    ~Executive()
    {
        disconnectAllTaps();
    }

    void run();

private:
    constexpr static unsigned int REFRESH_RATE_HZ = 5;
    constexpr static int GRID_SIZE = 60;

    void disconnectAllTaps();

    void planToGoal(Pose<> goal);
    void publishGoal();
    void publishInitialPose();

    void stepExecutiveFunctions();

    AStar<Grid2d> algorithm_;

    Grid2d grid_;

    ros::Publisher pubGoalPlan_;
    ros::Publisher pubGoalGoal_;
    ros::Publisher pubInitialPose_;

    Pose<> robotCurrentPose_;
    Pose<> robotInitialPose_;
    Pose<> currentGoal_;

    ros::NodeHandle rosNodeHandle_;

    RosTapCmd_Goal tapGoal_;
    RosTapCmd_YouAreHere tapYouAreHere_;

    int inc_;
};

} // namespace srs

#endif  // EXECUTIVE_HPP_

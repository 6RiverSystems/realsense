/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef EXECUTIVE_HPP_
#define EXECUTIVE_HPP_

#include <srslib_framework/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>

#include <srslib_framework/robotics/Pose.hpp>

#include <srslib_framework/ros/tap/RosTapMap.hpp>
#include <srslib_framework/ros/tap/RosTapInternal_GoalArrived.hpp>
#include <srslib_framework/ros/tap/RosTapInternal_RobotPose.hpp>
#include <srslib_framework/ros/tap/RosTapOperationalState.hpp>

#include <srslib_framework/search/AStar.hpp>

#include <srsnode_executive/tap/RosTapCmd_Goal.hpp>
#include <srsnode_executive/tap/RosTapCmd_InitialPose.hpp>
#include <srsnode_executive/tap/RosTapCmd_Move.hpp>
#include <srsnode_executive/tap/RosTapCmd_Pause.hpp>
#include <srsnode_executive/tap/RosTapCmd_Shutdown.hpp>

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
    constexpr static double REFRESH_RATE_HZ = 5;
    constexpr static int GRID_SIZE = 60;

    void connectAllTaps();

    void disconnectAllTaps();

    void findActiveNodes(vector<string>& nodes);

    void executeArrived();
    void executeInitialPose();
    void executePause();
    void executePlanToGoal();
    void executePlanToMove();
    void executeShutdown();
    void executeUnpause();

    void publishInternalInitialPose(Pose<> initialPose);
    void publishInternalGoalSolution(Solution<GridSolutionItem>* solution);
    void publishGoalTarget(Pose<> goalTargetArea);

    void stepExecutiveFunctions();

    AStar<Grid2d> algorithm_;
    bool arrived_;

    ros::Publisher pubExternalArrived_;
    ros::Publisher pubInternalInitialPose_;
    ros::Publisher pubInternalGoalSolution_;
    ros::Publisher pubStatusGoalPlan_;
    ros::Publisher pubStatusGoal_;
    ros::Publisher pubStatusGoalTarget_;

    Pose<> currentRobotPose_;
    Pose<> currentGoal_;
    Pose<> currentTarget_;
    Solution<GridSolutionItem>* currentSolution_;

    Pose<> robotInitialPose_;
    ros::NodeHandle rosNodeHandle_;

    RosTapCmd_Goal tapCmdGoal_;
    RosTapCmd_InitialPose tapCmdInitialPose_;
    RosTapCmd_Move tapCmdMove_;
    RosTapCmd_Shutdown tapCmdShutdown_;
    RosTapInternal_GoalArrived tapInternal_GoalArrived_;
    RosTapInternal_RobotPose tapInternal_RobotPose_;
    RosTapOperationalState tapOperationalState_;

    RosTapMap tapMap_;
};

} // namespace srs

#endif  // EXECUTIVE_HPP_

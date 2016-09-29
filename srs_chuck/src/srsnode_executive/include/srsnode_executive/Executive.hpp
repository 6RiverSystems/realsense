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
#include <srslib_framework/ros/publisher/PublisherBoolean.hpp>
#include <srslib_framework/ros/publisher/PublisherPolygonStamped.hpp>
#include <srslib_framework/ros/publisher/PublisherPose.hpp>
#include <srslib_framework/ros/publisher/PublisherPoseStamped.hpp>
#include <srslib_framework/ros/tap/RosTapMap.hpp>
#include <srslib_framework/ros/tap/RosTapInternal_GoalArrived.hpp>
#include <srslib_framework/ros/tap/RosTapInternal_RobotPose.hpp>
#include <srslib_framework/ros/tap/RosTapJoyAdapter.hpp>
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
    constexpr static double MAX_RELOCATION_THRESHOLD = 0.2;

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

    bool isExecutingSolution()
    {
        return !isJoystickLatched_ && !arrived_;
    }

    void publishGoalTarget(Pose<> goalTargetArea);
    void publishInternalInitialPose(Pose<> initialPose);

    void stepChecks();
    void stepExecutiveFunctions();

    void taskCustomAction();
    void taskPauseChange();
    void taskPlanToGoal();

	bool arrived_;

    bool buttonX_;

    Pose<> currentRobotPose_;
    Pose<> currentGoal_;
    Solution<GridSolutionItem>* currentSolution_;
    Pose<> currentTarget_;

    bool isJoystickLatched_;

    PublisherBoolean pubExternalArrived_;
    PublisherPose pubInternalInitialPose_;
    ros::Publisher pubStatusGoalPlan_;
    PublisherPose pubStatusGoal_;
    PublisherPolygonStamped pubStatusGoalTarget_;
    PublisherPoseStamped pubGoalToNavigation_;

    Pose<> robotInitialPose_;
    ros::NodeHandle rosNodeHandle_;

    RosTapCmd_Goal tapCmdGoal_;
    RosTapCmd_InitialPose tapCmdInitialPose_;
    RosTapCmd_Move tapCmdMove_;
    RosTapCmd_Shutdown tapCmdShutdown_;
    RosTapInternal_GoalArrived tapInternal_GoalArrived_;
    RosTapInternal_RobotPose tapInternal_RobotPose_;
    RosTapOperationalState tapOperationalState_;
	RosTapJoyAdapter tapJoyAdapter_;

    RosTapMap tapMap_;
};

} // namespace srs

#endif  // EXECUTIVE_HPP_

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef FACTORYROBOTPROFILE_HPP_
#define FACTORYROBOTPROFILE_HPP_

#include <opencv2/opencv.hpp>

#include <srsnode_motion/MotionConfig.h>
using namespace srsnode_motion;

#include <srslib_framework/robotics/robot/RobotProfile.hpp>

struct FactoryRobotProfile
{
    static RobotProfile fromConfiguration(MotionConfig& configuration)
    {
        RobotProfile robot;

        robot.emergencyKv = configuration.controller_emergency_kv;
        robot.emergencyKw = configuration.controller_emergency_kw;
        robot.emergencyMaxAngularVelocity = configuration.controller_emergency_max_angular_velocity;
        robot.emergencyMaxLinearVelocity = configuration.controller_emergency_max_linear_velocity;

        robot.manualKv = configuration.controller_manual_kv;
        robot.manualKw = configuration.controller_manual_kw;
        robot.manualMaxAngularVelocity = configuration.controller_manual_max_angular_velocity;
        robot.manualMaxLinearVelocity = configuration.controller_manual_max_linear_velocity;

        robot.physicalMaxAngularAcceleration = configuration.physical_max_angular_acceleration;
        robot.physicalMaxAngularVelocity = configuration.physical_max_angular_velocity;
        robot.physicalMaxLinearAcceleration = configuration.physical_max_linear_acceleration;
        robot.physicalMaxLinearVelocity = configuration.physical_max_linear_velocity;
        robot.physicalMinAngularVelocity = configuration.physical_min_angular_velocity;
        robot.physicalMinDistanceToGoal = configuration.physical_min_distance_to_goal;
        robot.physicalMinLinearVelocity = configuration.physical_min_linear_velocity;

        robot.pathFollowAdaptiveLookAhead = configuration.controller_pathfollow_adaptive_lookahead_enabled;
        robot.pathFollowGoalReachedDistance = configuration.controller_pathfollow_goal_reached_distance;
        robot.pathFollowKv = configuration.controller_pathfollow_kv;
        robot.pathFollowKw = configuration.controller_pathfollow_kw;
        robot.pathFollowLandingDepth = configuration.controller_pathfollow_landing_depth;
        robot.pathFollowLandingWidth = configuration.controller_pathfollow_landing_width;
        robot.pathFollowLinearAcceleration = configuration.controller_pathfollow_linear_acceleration;
        robot.pathFollowMaxAngularVelocity = configuration.controller_pathfollow_max_angular_velocity;
        robot.pathFollowMaxLinearVelocity = configuration.controller_pathfollow_max_linear_velocity;
        robot.pathFollowMaxLookAheadDistance = configuration.controller_pathfollow_max_look_ahead_distance;
        robot.pathFollowMinLinearVelocity = configuration.controller_pathfollow_min_linear_velocity;
        robot.pathFollowMinLookAheadDistance = configuration.controller_pathfollow_min_look_ahead_distance;
        robot.pathFollowSmallStraightDistance = configuration.controller_pathfollow_small_straight_distance;
        robot.pathFollowTurningVelocity = configuration.controller_pathfollow_turning_linear_velocity;
        robot.pathFollowTurningZoneRadius = configuration.controller_pathfollow_turning_zone_radius;
        robot.pathFollowZeroLookAheadDistance = configuration.controller_pathfollow_zero_look_ahead_distance;

        robot.rotationGoalReachedAngle = configuration.controller_rotation_goal_reached_angle;
        robot.rotationKd = configuration.controller_rotation_kd;
        robot.rotationKi = configuration.controller_rotation_ki;
        robot.rotationKp = configuration.controller_rotation_kp;
        robot.rotationKv = configuration.controller_rotation_kv;
        robot.rotationKw = configuration.controller_rotation_kw;
        robot.rotationMinAngularVelocity = configuration.controller_rotation_min_angular_velocity;
        robot.rotationRotationVelocity = configuration.controller_rotation_rotation_velocity;

        robot.singleCommandMode = configuration.single_command_mode;
        robot.stopMinLinearVelocity = configuration.controller_stop_min_linear_velocity;
        robot.stopNormalDeceleration = configuration.controller_stop_normal_linear_deceleration;

        return robot;
    }
};

#endif // FACTORYROBOTPROFILE_HPP_



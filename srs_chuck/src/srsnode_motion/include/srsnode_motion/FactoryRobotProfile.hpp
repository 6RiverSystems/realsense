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

#include <srslib_framework/robotics/RobotProfile.hpp>

struct FactoryRobotProfile
{
    static RobotProfile fromConfiguration(MotionConfig& configuration)
    {
        RobotProfile robot;

        robot.emergencyKv = configuration.emergency_controller_kv;
        robot.emergencyKw = configuration.emergency_controller_kw;
        robot.emergencyMaxAngularVelocity = configuration.emergency_controller_max_angular_velocity;
        robot.emergencyMaxLinearVelocity = configuration.emergency_controller_max_linear_velocity;

        robot.manualKv = configuration.manual_controller_kv;
        robot.manualKw = configuration.manual_controller_kw;
        robot.manualMaxAngularVelocity = configuration.manual_controller_max_angular_velocity;
        robot.manualMaxLinearVelocity = configuration.manual_controller_max_linear_velocity;

        robot.physicalMaxAngularAcceleration = configuration.physical_max_angular_acceleration;
        robot.physicalMaxAngularVelocity = configuration.physical_max_angular_velocity;
        robot.physicalMaxLinearAcceleration = configuration.physical_max_linear_acceleration;
        robot.physicalMaxLinearVelocity = configuration.physical_max_linear_velocity;
        robot.physicalMinAngularVelocity = configuration.physical_min_angular_velocity;
        robot.physicalMinLinearVelocity = configuration.physical_min_linear_velocity;

        robot.pathFollowAdaptiveLookAhead = configuration.pathfollow_controller_adaptive_lookahead_enabled;
        robot.pathFollowGoalReachedDistance = configuration.pathfollow_controller_goal_reached_distance;
        robot.pathFollowKv = configuration.pathfollow_controller_kv;
        robot.pathFollowKw = configuration.pathfollow_controller_kw;
        robot.pathFollowLandingDepth = configuration.pathfollow_controller_landing_depth;
        robot.pathFollowLandingWidth = configuration.pathfollow_controller_landing_width;
        robot.pathFollowLinearAcceleration = configuration.pathfollow_controller_linear_acceleration;
        robot.pathFollowMaxAngularVelocity = configuration.pathfollow_controller_max_angular_velocity;
        robot.pathFollowMaxLinearVelocity = configuration.pathfollow_controller_max_linear_velocity;
        robot.pathFollowMaxLookAheadDistance = configuration.pathfollow_controller_max_look_ahead_distance;
        robot.pathFollowMinLinearVelocity = configuration.pathfollow_controller_min_linear_velocity;
        robot.pathFollowMinLookAheadDistance = configuration.pathfollow_controller_min_look_ahead_distance;
        robot.pathFollowSmallStraightDistance = configuration.pathfollow_controller_small_straight_distance;
        robot.pathFollowTurningVelocity = configuration.pathfollow_controller_turning_linear_velocity;
        robot.pathFollowTurningZoneRadius = configuration.pathfollow_controller_turning_zone_radius;
        robot.pathFollowZeroLookAheadDistance = configuration.pathfollow_controller_zero_look_ahead_distance;

        robot.rotationGoalReachedAngle = configuration.rotation_controller_goal_reached_angle;
        robot.rotationKd = configuration.rotation_controller_kd;
        robot.rotationKi = configuration.rotation_controller_ki;
        robot.rotationKp = configuration.rotation_controller_kp;
        robot.rotationKv = configuration.rotation_controller_kv;
        robot.rotationKw = configuration.rotation_controller_kw;
        robot.rotationMinAngularVelocity = configuration.rotation_controller_min_angular_velocity;
        robot.rotationRotationVelocity = configuration.rotation_controller_rotation_velocity;

        robot.stopMinLinearVelocity = configuration.stop_controller_min_linear_velocity;
        robot.stopNormalDeceleration = configuration.stop_controller_normal_linear_deceleration;

        return robot;
    }
};

#endif // FACTORYROBOTPROFILE_HPP_



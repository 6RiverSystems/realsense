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

        robot.adaptiveLookAhead = configuration.adaptive_lookahead_enabled;

        robot.emergencyRatioCrawl = configuration.emergency_controller_ratio_crawl;
        robot.emergencyKv = configuration.emergency_controller_kv;
        robot.emergencyKw = configuration.emergency_controller_kw;

        robot.goalReachedDistance = configuration.goal_reached_distance;
        robot.goalReachedAngle = configuration.goal_reached_angle;

        robot.manualRatioAngular = configuration.manual_controller_ratio_angular;
        robot.manualRatioLinear = configuration.manual_controller_ratio_linear;
        robot.manualKv = configuration.manual_controller_kv;
        robot.manualKw = configuration.manual_controller_kw;
        robot.maxAngularAcceleration = configuration.max_angular_acceleration;
        robot.maxAngularVelocity = configuration.max_angular_velocity;
        robot.maxLinearAcceleration = configuration.max_linear_acceleration;
        robot.maxLinearVelocity = configuration.max_linear_velocity;
        robot.maxLookAheadDistance = configuration.max_look_ahead_distance;
        robot.minAngularVelocity = configuration.min_angular_velocity;
        robot.minLinearVelocity = configuration.min_linear_velocity;
        robot.minLookAheadDistance = configuration.min_look_ahead_distance;
        robot.minPhysicalAngularVelocity = configuration.min_physical_angular_velocity;
        robot.minPhysicalLinearVelocity = configuration.min_physical_linear_velocity;

        robot.pathFollowKv= configuration.pathfollow_controller_kw;
        robot.pathFollowKw= configuration.pathfollow_controller_kv;

        robot.rotationKd = configuration.rotation_controller_kd;
        robot.rotationKi = configuration.rotation_controller_ki;
        robot.rotationKp= configuration.rotation_controller_kp;
        robot.rotationKv= configuration.rotation_controller_kw;
        robot.rotationKw= configuration.rotation_controller_kv;

        robot.smallStraightDistance = configuration.small_straight_distance;

        robot.travelAngularAcceleration = configuration.travel_angular_acceleration;
        robot.travelAngularVelocity = configuration.travel_angular_velocity;
        robot.travelTurningZoneRadius = configuration.travel_turning_zone_radius;
        robot.travelTurningVelocity = configuration.travel_turning_linear_velocity;
        robot.travelLinearAcceleration = configuration.travel_linear_acceleration;
        robot.travelLinearVelocity = configuration.travel_linear_velocity;
        robot.travelRotationVelocity = configuration.travel_rotation_velocity;

        robot.zeroLookAheadDistance = configuration.zero_look_ahead_distance;

        return robot;
    }
};

#endif // FACTORYROBOTPROFILE_HPP_



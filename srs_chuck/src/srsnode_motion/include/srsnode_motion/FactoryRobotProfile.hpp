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

        robot.goalReachedDistance = configuration.goal_reached_distance;
        robot.goalReachedAngle = configuration.goal_reached_angle;

        robot.manualRatioAngular = configuration.manual_controller_ratio_angular;
        robot.manualRatioLinear = configuration.manual_controller_ratio_linear;
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

        robot.rotationKd = configuration.rotation_controller_kd;
        robot.rotationKi = configuration.rotation_controller_ki;
        robot.rotationKp= configuration.rotation_controller_kp;

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



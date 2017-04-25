/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <iostream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/LaserScan.h>

#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/device/RobotState.hpp>

namespace srs
{

class HeadOnCollisionReflex
{
public:
    HeadOnCollisionReflex();
    virtual ~HeadOnCollisionReflex();

    /**
     * Set the current velocity of the robot.
     * @param velocity the current velocity
     */
    void setVelocity(const Velocity<>& velocity)
    {
        latestVelocity_ = velocity;
    };

    /**
     * Set a new scan
     * @param scan the scan
     * @param type the type of the scan
     */
    void setLaserScan(const sensor_msgs::LaserScan& scan);

    /**
     * Check to see if a hard stop should be triggered.
     * Also sets a flag if hard stop was needed.
     * @return true if a hard stop should be triggered.
     */
    bool checkHardStop();

    /**
     * Set the robot's linear deceleration rate
     * @param rate the deceleration rate
     */
    void setLinearDecelerationRate(double rate)
    {
        nominalDecelRate_ = rate;
    };

    /**
     * Set the current state of the robot.
     * @param velocity the current velocity
     */
    void setRobotState(const RobotState& state)
    {
        robotState_ = state;
    };

    void setTimeWindow(double val)
    {
        timeWindow_ = val;
    };

    void setMinDistanceForStop(double val)
    {
        minDistanceForStop_ = val;
    };

    void setMinLinearVelocityForCheck(double val)
    {
        minLinearVelocityForCheck_ = val;
    };

    void setMaxRelativeTrackingVelocity(double val)
    {
        maxRelativeTrackingVelocity_ = val;
    };

    void setLidarSectorHalfWidth(double val)
    {
        lidarSectorHalfWidth_ = val;
    };

    void setMaxAngularVelocityForCheck(double val)
    {
        maxAngularVelocityForCheck_ = val;
    };

    /**
     * Sets whether the reflex should be disable when the robot is paused
     */
    void setDisableOnPause(bool val)
    {
        disableOnPause_ = val;
    }

private:
    double getClosestRangeInSector(const sensor_msgs::LaserScan& scan);

    /**
     * Helper struct for storing items in a map
     */
    struct DistanceVelBufferObj
    {
        DistanceVelBufferObj(double time, double range, Velocity<> vel)
        {
            time_ = time;
            range_ = range;
            vel_ = vel;
        }

        double range_ = -1;
        Velocity<> vel_ = Velocity<>::INVALID;
        double time_ = 0;
    };

    std::list<DistanceVelBufferObj> buffer_;

    Velocity<> latestVelocity_ = Velocity<>::INVALID;

    RobotState robotState_;

    // Param-settable constants
    double nominalDecelRate_ = 0.7;  // [m/s^2]

    double timeWindow_ = 0.5; // [s] Size of window to use for average velocity calculation
    double minDistanceForStop_ = 0.3; // [m] If the obstacle is going to be this close to the robot when it stops, hard stop

    double minLinearVelocityForCheck_ = 0.02; // [m/s] The minimum robot velocity to trigger a hardstop
    double maxRelativeTrackingVelocity_ = 3.0; // [m/s] the max relative velocity to consider
    double lidarSectorHalfWidth_ = 0.05; // [rad] Half width of the lidar sector in front of the bot to check
    double maxAngularVelocityForCheck_ = 0.5; // [rad/s] Maximum angular velocity of robot to use when checking

    bool disableOnPause_ = true;
};

} /* namespace srs */


/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <polyclipping/clipper.hpp>
#include <fstream>
#include <iostream>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>

namespace srs
{

typedef ClipperLib::IntPoint clPoint;
typedef ClipperLib::Path clPath;

class HardStopReflex
{
public:
    HardStopReflex();
    virtual ~HardStopReflex();

    /**
     * Set the current pose of the robot.
     * @param pose the current pose
     */
    void setPose(const Pose<> pose)
    {
        latestPose_ = pose;
    };

    /**
     * Set the pose of the lidar on the robot
     * @param pose the pose of the lidar on the robot
     */
    void setLidarPose(const tf::Transform pose)
    {
        lidarPose_ = pose;
    };

    /**
     * Set the current velocity of the robot.
     * @param velocity the current velocity
     */
    void setVelocity(const Velocity<>& velocity)
    {
        latestVelocity_ = velocity;
    };

    /**
     * Set a new lidar scan
     * @param scan the scan
     */
    void setLaserScan(const sensor_msgs::LaserScan& scan);

    /**
     * Check to see if a hard stop should be triggered.
     * Also sets a flag if hard stop was needed.
     * @return true if a hard stop should be triggered.
     */
    bool checkHardStop();

    /**
     * Check to see if the hard stop should be cleared.
     * @return true if the hard stop should be cleared.
     */
    bool checkForClear();

    /**
     * Get the danger zone for display in rviz
     * @return the danger zone
     */
    std::vector<Pose<>> getDangerZoneForDisplay() const;

    std::vector<Pose<>> getFailedDangerZoneForDisplay() const;
    std::vector<Pose<>> getFailedLaserScanForDisplay() const;


    /**
     * Set the robot's footprint
     * @param footprint the footprint
     */
    void setFootprint(const std::vector<Pose<>> footprint)
    {
        footprint_ = footprint;
    };

    /**
     * Set the robot's linear deceleration rate
     * @param rate the deceleration rate
     */
    void setLinearDecelerationRate(double rate)
    {
        nominalDecelRate_ = rate;
    };

    /**
     * Set the robot's angular deceleration rate
     * @param rate the deceleration rate
     */
    void setAngularDecelerationRate(double rate)
    {
        angularDecelRate_ = rate;
    };

private:
    /**
     * Determine if the danger zone has been violated
     * @return true if a hard stop is appropriate
     */
    bool checkForDangerZoneViolation();

    /**
     * Update the danger zone with the latest velocity data.
     * @return true if a new danger zone could be calculated
     */
    bool updateDangerZone();

    /**
     * Transforms the footprint to the given pose and adds to the group of polygons.
     * @param polygons the vector of polygons to which the new one will be added.
     * @param pose the pose to which the footprint should be transformed
     * @param footprint the footprint of the robot
     */
    void addPoseToPolygonStack(std::vector<clPath>& polygons, Pose<> pose, const std::vector<Pose<>>& footprint);

    /**
     * Calculates the unions of a stack of polygons
     * @param output the union of the polygons
     * @param polygons all of the polygons to union
     */
    void calculateUnion(clPath& output, std::vector<clPath>& polygons);

    void dumpDataToLog();

    Pose<> latestPose_ = Pose<>::INVALID;
    Velocity<> latestVelocity_ = Velocity<>::INVALID;
    tf::Transform lidarPose_ = tf::Transform::getIdentity();

    double nominalDecelRate_ = 0.7;  // m/s^2
    double angularDecelRate_ = 1.0;  // r/s^2

    int badPointsForStop_ = 3;

    std::vector<Pose<>> footprint_;

    clPath laserScan_;
    clPath dangerZone_;


    clPath failedDangerZone_;
    clPath failedLaserScan_;


    /**
     * Flag that is set if a hard stop has happened and the system is waiting for a clear
     */
    bool waitingForClear_ = false;

    int maxConsecutiveDangerZoneViolations_ = 1;
    int numConsecutiveDangerZoneViolations_ = 0;

    /**
     * Scale factor for converting meters to polygon coordinates.
     */
    constexpr static double CL_SCALE_FACTOR = 1000.0;
};

} /* namespace srs */


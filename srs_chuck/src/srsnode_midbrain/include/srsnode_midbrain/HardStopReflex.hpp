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

#include <polyclipping/clipper.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/RobotState.hpp>

namespace srs
{

typedef ClipperLib::IntPoint clPoint;
typedef ClipperLib::Path clPath;
typedef std::vector<clPath> PathVector;
typedef std::vector<Pose<>> PoseVector;

enum class LaserScanType
{
    LIDAR,
    DEPTH_CAMERA
};

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
     * Set the pose of a sensor on the robot
     * @param pose the pose of the sensor on the robot
     * @param type the type of scan
     */
    void setSensorPose(const tf::Transform pose, LaserScanType type)
    {
        createLaserMapEntryIfMissing(type);
        laserScansMap_[type].pose = pose;
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
     * Set the current state of the robot.
     * @param velocity the current velocity
     */
    void setRobotState(const RobotState& state)
    {
        robotState_ = state;
    };

    /**
     * Set a new scan
     * @param scan the scan
     * @param type the type of the scan
     */
    void setLaserScan(const sensor_msgs::LaserScan& scan, LaserScanType type);

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
    PoseVector getDangerZoneForDisplay() const;

    /**
     * Get the latest danger zone that failed
     * @return the danger zone
     */
    PoseVector getFailedDangerZoneForDisplay() const;

    /**
     * Get the laser scan that penetrated the danger zone for display in rviz
     * @return the laser scan
     */
    PoseVector getFailedLaserScanForDisplay() const;


    /**
     * Set the robot's footprint
     * @param footprint the footprint
     */
    void setFootprint(const PoseVector footprint)
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

    /**
     * Sets the number of times the danger zone must be violated consecutively to trigger a hard stop.
     * @param val the limit
     */
    void setMaxConsecutiveDangerZoneViolations(uint32_t val)
    {
        maxConsecutiveDangerZoneViolations_ = val;
    }

    /**
     * Sets the number of scan points in the danger zone to be considered a violation
     */
    void setNumBadPointsForViolation(uint32_t val)
    {
        numBadPointsForViolation_ = val;
    }

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
    void addPoseToPolygonStack(PathVector& polygons, Pose<> pose, const PoseVector& footprint);

    /**
     * Calculates the unions of a stack of polygons
     * @param output the union of the polygons
     * @param polygons all of the polygons to union
     */
    void calculateUnion(clPath& output, PathVector& polygons);

    /**
     * Convert a clPath to a vector of poses
     * @param path the path to convert
     * @return a vector of poses
     */
    PoseVector clPathToPoseVector(const clPath& path) const;

    /**
     * Write some debug information to std::cout
     */
    void dumpDataToLog();

    /**
     * Create an entry in the laser map if it is missing.
     * @param type the type of the scan
     */
    void createLaserMapEntryIfMissing(LaserScanType type);

    /**
     * Helper struct for storing items in a map
     */
    struct LaserScanMapItem
    {
        tf::Transform pose = tf::Transform::getIdentity();
        clPath scan;
    };

    Pose<> latestPose_ = Pose<>::INVALID;
    Velocity<> latestVelocity_ = Velocity<>::INVALID;

    double nominalDecelRate_ = 0.7;  // m/s^2
    double angularDecelRate_ = 1.0;  // r/s^2

    uint32_t numBadPointsForViolation_ = 2;

    uint32_t maxConsecutiveDangerZoneViolations_ = 1;
    uint32_t numConsecutiveDangerZoneViolations_ = 0;

    PoseVector footprint_;

    clPath dangerZone_;

    clPath failedDangerZone_;
    clPath failedLaserScan_;

    std::map<LaserScanType, LaserScanMapItem> laserScansMap_;

    RobotState robotState_;

    /**
     * Flag that is set if a hard stop has happened and the system is waiting for a clear
     */
    bool waitingForClear_ = false;

    /**
     * Scale factor for converting meters to polygon coordinates.
     */
    constexpr static double CL_SCALE_FACTOR = 1000.0;

    /**
     * Epsilon to check if velocity is equal to 0
     */
    constexpr static double VELOCITY_EPSILON = 0.005;
};

} /* namespace srs */


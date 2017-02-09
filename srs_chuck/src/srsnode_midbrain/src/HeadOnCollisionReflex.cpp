/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */


#include <srsnode_midbrain/HeadOnCollisionReflex.hpp>

#include <ros/ros.h>

#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs
{

HeadOnCollisionReflex::HeadOnCollisionReflex()
{

}

HeadOnCollisionReflex::~HeadOnCollisionReflex()
{

}

void HeadOnCollisionReflex::setLaserScan(const sensor_msgs::LaserScan& scan)
{
    // Buffer the scans with the robot's velocity and current time stamp.
    // Throw out any that are too old.
    DistanceVelBufferObj obj(scan.header.stamp.toSec(), getClosestRangeInSector(scan), latestVelocity_);
    // Stick inthe buffer.
    buffer_.push_back(obj);
    // Remove any poses that are too old.
    buffer_.erase(
        std::remove_if(
            buffer_.begin(),
            buffer_.end(),
            [&](const DistanceVelBufferObj& ptr)
                {return (obj.time_ - ptr.time_) > timeWindow_;}
            ),
        buffer_.end()
    );
}

bool HeadOnCollisionReflex::checkHardStop()
{
    // Check the buffer and compare the distance
    if (buffer_.size() < 2)
    {
        return false;
    }

    if (latestVelocity_.linear < minLinearVelocityForCheck_)
    {
        return false;
    }

    // Go frame by frame
    std::list<double> ranges;
    double totalRangeChange = 0;
    double expectedTotalRangeChange = 0;
    auto currIt = buffer_.begin();
    auto nextIt = buffer_.begin();
    ++nextIt;
    ranges.push_back(currIt->range_);
    for (; nextIt != buffer_.end(); ++currIt, ++nextIt)
    {
        ranges.push_back(nextIt->range_);
        if (nextIt->range_ <= 0 || currIt->range_ <= 0)
        {
            // If the ranges are 0, ignore the values
            return false;
        }
        // Calculate the change in obstacle distance for nearest in scan sector.
        double dRange = nextIt->range_ - currIt->range_;
        // If it increases during the window, return.
        if (dRange >= 0)
        {
            // Range has increased. No need to hard stop.
            return false;
        }
        // If it decreases, use the average velocity of the two frames for the dt between
        //    calculate the expected difference between two scans.
        double dt = nextIt->time_ - currIt->time_;
        if (dt <= 0)
        {
            ROS_WARN("Invalid dt in Head on collision calc of %f", dt);
            return false;
        }
        // Make sure the velocity makes sense
        if (std::fabs(dRange / dt) > maxRelativeTrackingVelocity_)
        {
            return false;
        }

        if (nextIt->vel_.linear <= 0 || currIt->vel_.linear <= 0)
        {
            ROS_DEBUG("No head on collision if the velocities are negative or 0");
            return false;
        }

        if (std::fabs(nextIt->vel_.angular) > maxAngularVelocityForCheck_
            || std::fabs(currIt->vel_.angular) > maxAngularVelocityForCheck_)
        {
            ROS_DEBUG("No head on collision if the angular velocities are too great");
            return false;
        }
        double aveV = (nextIt->vel_.linear + currIt->vel_.linear) / 2;
        double expectedDiffRange = aveV * dt;
        totalRangeChange += -dRange; // store as a positive value
        expectedTotalRangeChange += expectedDiffRange;
    }
    // Sum up the total change vs. the total expected.  Use to calculate average velocity.
    double totalDT = buffer_.back().time_ - buffer_.front().time_;
    if (totalDT <= 0)
    {
        ROS_WARN("Cannot have a non-positive time difference in HOCR laser scan buffer");
        return false;
    }
    double rangeChangeOverExpected = totalRangeChange - expectedTotalRangeChange;
    if (rangeChangeOverExpected < 0)
    {
        return false;
    }
    double obstacleAverageVelocity = rangeChangeOverExpected / totalDT;

    // Given current velocity, calculate stopping time and distance nominal.
    if (nominalDecelRate_ <= 0)
    {
        ROS_WARN("Head on collision reflex cannot have a non-positive deceleration rate.");
        return false;
    }
    double stoppingTime = latestVelocity_.linear / nominalDecelRate_; // time until the robot could stop
    double stoppingDistance = 0.5 * latestVelocity_.linear * stoppingTime; // how far it will go in that time

    // Given the velocity and range to the obstacle, see if it will get within a threshold of the sensor.
    double currentObstacleRange = buffer_.back().range_;
    double obstacleRangeAtStop = currentObstacleRange - (stoppingTime * obstacleAverageVelocity);

    if (obstacleRangeAtStop < minDistanceForStop_)
    {
        ROS_WARN("Hard stopping to avoid head on collision.  totalDT %f, totalRangeChange: %f, expectedTotalRangeChange: %f, stoppingTime %f, stoppingDistance %f",
            totalDT, totalRangeChange, expectedTotalRangeChange, stoppingTime, stoppingDistance);
        std::stringstream ss;
        for (auto r : ranges)
        {
            ss << r << ", ";
        }
        ROS_INFO("Head on collision ranges: %s", ss.str().c_str());
        return true;
    }

    // If we make it to here, don't hardstop
    return false;
}

double HeadOnCollisionReflex::getClosestRangeInSector(const sensor_msgs::LaserScan& scan)
{
    // For now, we are assuming that the laser scanner is centered in the front of the bot
    double range = -1;
    for (size_t k = 0; k < scan.ranges.size(); ++k)
    {
        double angle = k * scan.angle_increment + scan.angle_min;
        if (angle >= -lidarSectorHalfWidth_ && angle < lidarSectorHalfWidth_)
        {
            double rayRange = scan.ranges[k];
            if (rayRange < scan.range_min || rayRange > scan.range_max)
            {
                continue;
            }
            if (range < 0 || rayRange < range)
            {
                range = rayRange;
            }
        }
    }
    return range;
}

} /* namespace srs */


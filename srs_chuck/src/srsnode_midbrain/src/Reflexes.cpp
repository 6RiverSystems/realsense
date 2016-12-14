/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>

#include <costmap_2d/footprint.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PolygonStamped.h>
#include <tgmath.h>
#include <boost/range/adaptor/reversed.hpp>

#include <srslib_framework/ros/message/PolygonMessageFactory.hpp>
#include <srslib_framework/ros/message/PoseMessageFactory.hpp>

namespace srs
{

Reflexes::Reflexes()
{
    readParams();
}

Reflexes::~Reflexes( )
{

}

void Reflexes::readParams()
{
    // Get the footprint
    ros::NodeHandle costmapNh("/move_base/local_costmap");

    std::vector<geometry_msgs::Point> footprint = costmap_2d::makeFootprintFromParams(costmapNh);
    hardStopReflex_.setFootprint(PolygonMessageFactory::points2Poses(footprint));

    // Get other params
    ros::NodeHandle privateNh("~");
    double linRate = 0.0;
    if (privateNh.getParam("linear_deceleration_rate", linRate))
    {
    	hardStopReflex_.setLinearDecelerationRate(linRate);
    }
    double angRate = 0.0;
    if (privateNh.getParam("angular_deceleration_rate", angRate))
    {
    	hardStopReflex_.setAngularDecelerationRate(angRate);
    }
    int maxDZViolations = 0;
    if (privateNh.getParam("max_consecutive_danger_zone_violations", maxDZViolations))
    {
    	hardStopReflex_.setMaxConsecutiveDangerZoneViolations(maxDZViolations);
    }
    int numBadPoints = 0;
    if (privateNh.getParam("num_bad_points_for_violation", numBadPoints))
    {
    	hardStopReflex_.setNumBadPointsForViolation(numBadPoints);
    }
}

void Reflexes::execute()
{
    // Get the sensor position
    hardStopReflex_.setLidarPose(tapLidarPoseOnRobot_.pop());

    // Update robot position
    hardStopReflex_.setPose(tapRobotPose_.pop());

    // Check for laser scan.
    if (tapFilteredLidar_.newDataAvailable())
    {
        hardStopReflex_.setLaserScan(tapFilteredLidar_.pop());
    }

    // Check for velocity.
    if (tapOdometryPose_.newDataAvailable())
    {
        hardStopReflex_.setVelocity(tapOdometryPose_.popVelocity());
    }

    // Call for hard stop check.
    if (hardStopReflex_.checkHardStop())
    {
    	srslib_framework::MsgSetOperationalState setOperationalState;
    	setOperationalState.state = true;
    	setOperationalState.operationalState.hardStop = true;

        ROS_WARN_THROTTLE(1.0f, "Publishing stop");
        setMotionStateChannel_.publish(setOperationalState);
    }
    else if (hardStopReflex_.checkForClear())
    {
       	srslib_framework::MsgSetOperationalState setOperationalState;
		setOperationalState.state = false;
		setOperationalState.operationalState.hardStop = true;

		ROS_WARN_THROTTLE(1.0f, "Clearing motion status stop");
		setMotionStateChannel_.publish(setOperationalState);
   }

    // Publish the polygon
    dangerZoneChannel_.publish(hardStopReflex_.getDangerZoneForDisplay());

    // Debugging help
    if (enableHardStopDebugPlotting_)
    {
        auto fdz = hardStopReflex_.getFailedDangerZoneForDisplay();
        if (fdz.size() > 0)
        {
            fdzChannel_.publish(fdz);
        }
        auto fls = hardStopReflex_.getFailedLaserScanForDisplay();
        if (fls.size() > 0)
        {
            flsChannel_.publish(fls);
        }
    }
}

} /* namespace srs */


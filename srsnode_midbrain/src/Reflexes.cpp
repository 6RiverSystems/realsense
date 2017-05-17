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

#include <srslib_timing/ScopedTimingSampleRecorder.hpp>

namespace srs
{

Reflexes::Reflexes() : tdr_("Reflexes")
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
    //  Get the hard stop reflex params
    ros::NodeHandle privateNh("~");
    double linRate = 0.0;
    if (privateNh.getParam("hard_stop_reflex/linear_deceleration_rate", linRate))
    {
    	hardStopReflex_.setLinearDecelerationRate(linRate);
    }
    double angRate = 0.0;
    if (privateNh.getParam("hard_stop_reflex/angular_deceleration_rate", angRate))
    {
    	hardStopReflex_.setAngularDecelerationRate(angRate);
    }
    double scanTimeout = 0.0;
    if (privateNh.getParam("hard_stop_reflex/scan_timeout", scanTimeout))
    {
        hardStopReflex_.setScanTimeout(scanTimeout);
    }
    int maxDZViolations = 0;
    if (privateNh.getParam("hard_stop_reflex/max_consecutive_danger_zone_violations", maxDZViolations))
    {
    	hardStopReflex_.setMaxConsecutiveDangerZoneViolations(maxDZViolations);
    }
    int numBadPoints = 0;
    if (privateNh.getParam("hard_stop_reflex/num_bad_points_for_violation", numBadPoints))
    {
    	hardStopReflex_.setNumBadPointsForViolation(numBadPoints);
    }

    bool checkLidarHardStop = true;
    if (privateNh.getParam("hard_stop_reflex/check_lidar_hard_stop", checkLidarHardStop))
    {
        hardStopReflex_.enableLaserScanType(LaserScanType::LIDAR, checkLidarHardStop);
    }
    bool checkDepthCameraHardStop = true;
    if (privateNh.getParam("hard_stop_reflex/check_depth_camera_hard_stop", checkDepthCameraHardStop))
    {
        hardStopReflex_.enableLaserScanType(LaserScanType::DEPTH_CAMERA, checkDepthCameraHardStop);
    }
    bool hsrDisableOnPause = true;
    if (privateNh.getParam("hard_stop_reflex/disable_on_pause", hsrDisableOnPause))
    {
        hardStopReflex_.setDisableOnPause(hsrDisableOnPause);
    }

    // Get the head on collision reflex params
    double val = 0.0;
    if (privateNh.getParam("head_on_collision_reflex/linear_decel_rate", val))
    {
        headOnCollisionReflex_.setLinearDecelerationRate(val);
    }
    if (privateNh.getParam("head_on_collision_reflex/time_window", val))
    {
        headOnCollisionReflex_.setTimeWindow(val);
    }
    if (privateNh.getParam("head_on_collision_reflex/min_distance_for_stop", val))
    {
        headOnCollisionReflex_.setMinDistanceForStop(val);
    }
    if (privateNh.getParam("head_on_collision_reflex/min_linear_velocity_for_check", val))
    {
        headOnCollisionReflex_.setMinLinearVelocityForCheck(val);
    }
    if (privateNh.getParam("head_on_collision_reflex/max_relative_tracking_velocity", val))
    {
        headOnCollisionReflex_.setMaxRelativeTrackingVelocity(val);
    }
    if (privateNh.getParam("head_on_collision_reflex/lidar_sector_half_width", val))
    {
        headOnCollisionReflex_.setLidarSectorHalfWidth(val);
    }
    if (privateNh.getParam("head_on_collision_reflex/max_angular_velocity_for_check", val))
    {
        headOnCollisionReflex_.setMaxAngularVelocityForCheck(val);
    }
    bool hocrDisableOnPause = true;
    if (privateNh.getParam("head_on_collision_reflex/disable_on_pause", hocrDisableOnPause))
    {
        headOnCollisionReflex_.setDisableOnPause(hocrDisableOnPause);
    }
}

void Reflexes::execute()
{
    srs::ScopedTimingSampleRecorder stsr(tdr_.getRecorder("-Loop"));

    // Update robot position
    hardStopReflex_.setPose(tapRobotPose_.pop());

    // Update brainstem connected state
    brainstemConnected_ = tapBrainstemConnected_.pop();

    // Update the robot state
    if (tapOperationalState_.newDataAvailable())
    {
        hardStopReflex_.setRobotState(tapOperationalState_.peek());
        headOnCollisionReflex_.setRobotState(tapOperationalState_.pop());
    }

    // Check for laser scan.
    if (tapFilteredLidar_.newDataAvailable())
    {
        headOnCollisionReflex_.setLaserScan(tapFilteredLidar_.peek());
        hardStopReflex_.setLaserScan(tapFilteredLidar_.pop(), LaserScanType::LIDAR);
    }

    // Get the lidar sensor position
    hardStopReflex_.setSensorPose(tapLidarPoseOnRobot_.pop(), LaserScanType::LIDAR);

    // Get the camera sensor position
    hardStopReflex_.setSensorPose(tapDepthCameraPoseOnRobot_.pop(), LaserScanType::DEPTH_CAMERA);

    // Check for camera scan.
    if (tapFilteredDepthCamera_.newDataAvailable())
    {
        hardStopReflex_.setLaserScan(tapFilteredDepthCamera_.pop(), LaserScanType::DEPTH_CAMERA);
    }

    // Check for velocity.
    if (tapOdometryPose_.newDataAvailable())
    {
        headOnCollisionReflex_.setVelocity(tapOdometryPose_.peekVelocity());
        hardStopReflex_.setVelocity(tapOdometryPose_.popVelocity());
    }

    if (brainstemConnected_)
    {
		// Call for hard stop check.
		if (hardStopReflex_.checkHardStop(ros::Time::now().toSec()) || headOnCollisionReflex_.checkHardStop())
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


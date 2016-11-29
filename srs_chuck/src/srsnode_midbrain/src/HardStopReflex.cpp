/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>
#include <boost/geometry/algorithms/union.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tgmath.h>

namespace srs
{

HardStopReflex::HardStopReflex()
{

}

HardStopReflex::~HardStopReflex()
{

}

void HardStopReflex::setLidarPose(Pose<> pose)
{
	lidarPose_ = pose;
}

void HardStopReflex::setPose(Pose<> pose)
{
	latestPose_ = pose;
}

void HardStopReflex::setLaserScan(const sensor_msgs::LaserScan& scan)
{
	if (!(latestPose_.isValid() && lidarPose_.isValid()))
	{
		ROS_WARN("Cannot save the laser scan without valid transforms.");
		return;
	}
	Pose<> lidarToMap = PoseMath::multiply(latestPose_, lidarPose_);
	ROS_DEBUG("Processing scan at: %f, %f, %f", lidarToMap.x, lidarToMap.y, lidarToMap.theta);
	// Iterate over the scan and convert into the map frame.
	laserScan_.clear();
	laserScan_.reserve(scan.ranges.size());
	uint32_t numberOfPoints = scan.ranges.size();
	for (size_t k = 0; k < scan.ranges.size(); ++k)
	{
		double angle = ((double)k * scan.angle_increment) + scan.angle_min;
		double scanDistance = scan.ranges[k];

		if (std::isfinite(scanDistance) && scanDistance >= scan.range_min)
		{
			// Convert the scan data into an x,y point in the map
			double x = cos(angle) * scanDistance;
			double y = sin(angle) * scanDistance;
			Pose<> pt = Pose<>(x, y);
			Pose<> ptInMap = PoseMath::multiply(lidarToMap, pt);
			ROS_DEBUG("x: %f, y: %f, in map: x: %f, y: %f", x, y, ptInMap.x, ptInMap.y);

			// Now convert the data into a bg point
			laserScan_.push_back(Point(ptInMap.x, ptInMap.y));
		}
	}
}

bool HardStopReflex::checkHardStop()
{
	if (!updateDangerZone())
	{
		ROS_WARN("Could not update danger zone.  Return true for hard stop.");
		hardStopActivated_ = true;
		waitingForClear_ = true;
		return true;
	}

	if (VelocityMath::equal(latestVelocity_, Velocity<>::ZERO))
	{
		ROS_DEBUG("Velocity is zero, no need to trigger the estop.");
		hardStopActivated_ = false;
		return false;
	}

	// Iterate over the points.
	for (auto pt : laserScan_)
	{
		if (boost::geometry::within(pt, dangerZone_))
		{
			ROS_WARN("Found a point in the danger zone at [%f, %f].  Robot at [%f, %f, th: %f].",
				pt.x, pt.y, latestPose_.x, latestPose_.y, latestPose_.theta);
			hardStopActivated_ = true;
			waitingForClear_ = true;
			return true;
		}
	}

	hardStopActivated_ = false;
	return false;
}

bool HardStopReflex::updateDangerZone()
{
	if (footprint_.size() < 3)
	{
		ROS_WARN("Cannot update danger zone without a footprint.");
		return false;
	}

	if (!latestVelocity_.isValid())
	{
		ROS_WARN("Cannot update danger zone.  Invalid velocity.");
		return false;
	}

	if (!latestPose_.isValid())
	{
		ROS_WARN("Cannot update danger zone.  Invalid pose.");
		return false;
	}

	// Clear the danger zone
	dangerZone_ = Polygon();

	// First pose is the current pose.
	Pose<> currentPose = latestPose_;
	double v = latestVelocity_.linear;
	double w = latestVelocity_.angular;
	double simDt = 0.01;
	double sampleDt = 0.1;
	double lastSaveTime = 0.0;
	double time = 0.0;

	double maxDv = simDt * nominalDecelRate_;
	double maxDw = simDt * angularDecelRate_;

	addPoseToPolygon(dangerZone_, currentPose, footprint_);
	while (!(BasicMath::equal(v, 0.0, 0.001) && BasicMath::equal(w, 0.0, 0.001)))
	{
		// Forward propogate
		Pose<> newPose = PoseMath::translate(currentPose, v * simDt, 0.0);
		currentPose = PoseMath::rotate(newPose, w * simDt);
		time += simDt;
		// Propogate the deceleration
		//  This will change depending on the firmware's algorithm
		double dV = 0.0;
		double dW = 0.0;

		if (!BasicMath::equal(v, 0.0, 0.001)) {
			dV = maxDv;
			if (std::fabs(v) - dV <= 0) {
				dV = std::fabs(v);
			}
		}

		if (!BasicMath::equal(w, 0.0, 0.001)) {
			// If there is a velocity change, calculate the angular velocity change
			if (!BasicMath::equal(dV, 0.0, 0.001))
			{
				dW = std::min(maxDw, dV * std::fabs(w / v));
			}
			else
			{
				dW = maxDw;
			}

			// Ensure that it will not yield a bad result
			if (std::fabs(w) - dW <= 0) {
				dW = std::fabs(w);
			}

			// Finally, rescale the linear acceleration to keep the arc
			dV = dW * std::fabs(v / w);
		}


		dV *= BasicMath::sgn(v);
		dW *= BasicMath::sgn(w);

		v -= dV;
		w -= dW;
		ROS_DEBUG("Updating with velocity: v: %f, w: %f, dV: %f, dW: %f", v, w, dV, dW);

		if (time - lastSaveTime >= sampleDt)
		{
			ROS_DEBUG("Adding a new pose to the polygon.  x: %f, y: %f, the: %f", currentPose.x, currentPose.y, currentPose.theta);
			addPoseToPolygon(dangerZone_, currentPose, footprint_);
			lastSaveTime = time;
		}
	}
	// Finally, put in the last pose.
	addPoseToPolygon(dangerZone_, currentPose, footprint_);
	return true;
}

void HardStopReflex::addPoseToPolygon(Polygon& polygon, Pose<> pose, const std::vector<Pose<>>& footprint)
{
	// Convert the footprint points to the pose
	Ring footprintPoints;

	// std::cout << "New footprint points: " << std::endl;
	for (auto p : footprint)
	{
		Pose<> point = PoseMath::multiply(pose, p);
		footprintPoints.push_back({point.x, point.y});
		// std::cout << "  x: " << point.x << ", y: " << point.y << std::endl;
	}
	// std::cout << "done" << std::endl;

	Polygon footprintPolygon;
	boost::geometry::assign_points( footprintPolygon, footprintPoints );

	std::vector<Polygon> combinedZone;

	// Normalize the polygons (if not the union function will not work properly)
	bg::correct( footprintPolygon );
	bg::correct( polygon );

	try
	{
		// Create the combined polygon by creating a union of the pose polygon and the input polygon
		bg::union_( footprintPolygon, polygon, combinedZone );
	}
	catch (...)
	{
		ROS_WARN("Caught exception when calculating new polygon.");
		return;
	}
	assert( combinedZone.size( ) );

	if ( combinedZone.size( ) )
	{
		polygon = combinedZone[0];
	}
}

bool HardStopReflex::checkForClear()
{
	// if (waitingForClear_ && !hardStopActivated_)
	if (waitingForClear_
		&& BasicMath::equal(latestVelocity_.linear, 0.0, 0.005)
		&& BasicMath::equal(latestVelocity_.angular, 0.0, 0.005))
	{
		ROS_INFO("Hard stop cleared ");
		waitingForClear_ = false;
		return true;
	}

	return false;
}

std::vector<Pose<>> HardStopReflex::getDangerZoneForDisplay() const
{
	std::vector<Pose<>> out;
	if (dangerZone_.outer().size() == 0)
	{
		ROS_WARN("No danger zone yet.");
		return out;
	}

	out.reserve(dangerZone_.outer().size());
	for (auto pt : dangerZone_.outer())
	{
		out.push_back(Pose<>(pt.x, pt.y, 0.0));
	}
}

} /* namespace srs */


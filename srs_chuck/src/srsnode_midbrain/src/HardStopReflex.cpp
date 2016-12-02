/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>

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

void HardStopReflex::setLaserScan(const sensor_msgs::LaserScan& scan)
{
	if (!latestPose_.isValid())
	{
		ROS_WARN("Cannot save the laser scan without valid transforms.");
		return;
	}
	tf::Transform lidarToMap = PoseMessageFactory::pose2Transform(latestPose_) * lidarPose_;

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
			tf::Transform pt = PoseMessageFactory::pose2Transform(Pose<>(x, y));
			Pose<> ptInMap = PoseMessageFactory::transform2Pose(lidarToMap * pt);
			ROS_DEBUG("x: %f, y: %f, in map: x: %f, y: %f", x, y, ptInMap.x, ptInMap.y);

			// Now convert the data into a bg point
			laserScan_.push_back(clPoint(ptInMap.x * CL_SCALE_FACTOR, ptInMap.y * CL_SCALE_FACTOR));
		}
	}
}

bool HardStopReflex::checkHardStop()
{
	bool dangerZoneViolation = checkForDangerZoneViolation();
	if (dangerZoneViolation)
	{
		numConsecutiveDangerZoneViolations_++;
	}
	else
	{
		numConsecutiveDangerZoneViolations_ = 0;
	}

	bool shouldHardStop = numConsecutiveDangerZoneViolations_ > maxConsecutiveDangerZoneViolations_;

	if (shouldHardStop)
	{
		waitingForClear_ = true;
	}
	return shouldHardStop;
}

bool HardStopReflex::checkForDangerZoneViolation()
{
	if (!updateDangerZone())
	{
		ROS_WARN_THROTTLE(1.0, "Could not update danger zone.  Return true for hard stop.");
		return true;
	}

	if (VelocityMath::equal(latestVelocity_, Velocity<>::ZERO))
	{
		ROS_DEBUG("Velocity is zero, no need to trigger the estop.");
		return false;
	}

	if (latestVelocity_.linear < 0 && std::fabs(latestVelocity_.angular) < 1.00)
	{
		ROS_DEBUG("Linear velocity is negative, no need to trigger the estop.");
		return false;
	}

	// Iterate over the points.
	int numBadPoints = 0;
	for (auto pt : laserScan_)
	{
		if (ClipperLib::PointInPolygon(pt, dangerZone_))
		{
			ROS_DEBUG("Found a point in the danger zone at [%f, %f].  Robot at [%f, %f, th: %f].",
				pt.X / CL_SCALE_FACTOR, pt.Y / CL_SCALE_FACTOR, latestPose_.x, latestPose_.y, latestPose_.theta);
			numBadPoints++;
		}
	}
	if (numBadPoints > 0)
	{
		ROS_DEBUG("Saw %d bad points of %d max", numBadPoints, numBadPointsForViolation_);
		failedDangerZone_ = dangerZone_;
		failedLaserScan_ = laserScan_;
	}
	return numBadPoints >= numBadPointsForViolation_;
}

void HardStopReflex::dumpDataToLog()
{

	std::cout << "Data dump after hard stop request: " << std::endl;
	std::cout << "Pose: " << latestPose_.x << ", " << latestPose_.y << ", " << latestPose_.theta << std::endl;

	std::cout << "Danger Zone: " << std::endl;
	for (auto p : dangerZone_)
	{
		std::cout << "  " << p.X << ", " << p.Y << std::endl;
	}

	std::cout << "Scan on map: " << std::endl;
	for (auto p : laserScan_)
	{
		std::cout << "  " << p.X << ", " << p.Y << std::endl;
	}

}

bool HardStopReflex::updateDangerZone()
{
	if (footprint_.size() < 3)
	{
		ROS_WARN_THROTTLE(1.0, "Cannot update danger zone without a footprint.");
		return false;
	}

	if (!latestVelocity_.isValid())
	{
		ROS_WARN_THROTTLE(1.0, "Cannot update danger zone.  Invalid velocity.");
		return false;
	}

	if (!latestPose_.isValid())
	{
		ROS_WARN_THROTTLE(1.0, "Cannot update danger zone.  Invalid pose.");
		return false;
	}

	// Clear the danger zone
	dangerZone_ = clPath();

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

	std::vector<clPath> footprints;
	addPoseToPolygonStack(footprints, currentPose, footprint_);

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
			addPoseToPolygonStack(footprints, currentPose, footprint_);
			lastSaveTime = time;
		}
	}
	// Finally, put in the last pose.
	addPoseToPolygonStack(footprints, currentPose, footprint_);
	calculateUnion(dangerZone_, footprints);
	return true;
}

void HardStopReflex::addPoseToPolygonStack(std::vector<clPath>& polygons, Pose<> pose, const std::vector<Pose<>>& footprint)
{
	// Convert the footprint points to the pose
	clPath footprintPolygon;

	// std::cout << "New footprint points: " << std::endl;
	for (auto p : footprint)
	{
		Pose<> point = PoseMath::multiply(pose, p);
		footprintPolygon.push_back(clPoint(point.x * 1000, point.y * 1000));
	}
	polygons.push_back(footprintPolygon);
}

void HardStopReflex::calculateUnion(clPath& output, std::vector<clPath>& polygons)
{
	ClipperLib::Clipper c;
	for (auto p : polygons)
	{
		c.AddPath(p, ClipperLib::ptSubject, true);
	}
	ClipperLib::Paths paths;
	c.Execute(ClipperLib::ctUnion, paths, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
	if (paths.size() > 0)
	{
		output = paths[0];
	}
	else
	{
		ROS_WARN("Did not get a solution to the union of footprint polygons.");
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
	return clPathToPoseVector(dangerZone_);
}

std::vector<Pose<>> HardStopReflex::getFailedDangerZoneForDisplay() const
{
	return clPathToPoseVector(failedDangerZone_);
}

std::vector<Pose<>> HardStopReflex::getFailedLaserScanForDisplay() const
{
	return clPathToPoseVector(failedLaserScan_);
}

std::vector<Pose<>> HardStopReflex::clPathToPoseVector(const clPath& path) const
{
	std::vector<Pose<>> out;
	if (path.size() == 0)
	{
		return out;
	}

	out.reserve(path.size());
	for (auto pt : path)
	{
		out.push_back(Pose<>(pt.X / CL_SCALE_FACTOR, pt.Y / CL_SCALE_FACTOR, 0.0));
	}
}

} /* namespace srs */


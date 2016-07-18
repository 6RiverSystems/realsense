/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MIDBRAIN_OBSTACLE_DETECTOR_HPP_
#define MIDBRAIN_OBSTACLE_DETECTOR_HPP_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>

namespace srs
{

class ObstacleDetector
{
	typedef std::function<void()> ObstacleDetectedFn;

public:
	ObstacleDetector( double footprint );
	virtual ~ObstacleDetector( );

	void SetDetectionCallback( ObstacleDetectedFn obstacleDetectedCallback );

	void SetVelocity( double linear, double angular );

	void SetObjectThreshold( uint32_t objectThreshold );

	void ProcessScan( const sensor_msgs::LaserScan::ConstPtr& scan );

	double GetSafeDistance( double velocity ) const;

	double GetFootprint( ) const;

private:

	ObstacleDetectedFn	m_obstacleDetectedCallback;

	double				m_linearVelocity;

	double				m_angularVelocity;

	uint32_t 			m_objectThreshold;

	double				m_footprint;

};

} /* namespace srs */

#endif /* MIDBRAIN_OBSTACLE_DETECTOR_HPP_ */

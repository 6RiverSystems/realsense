/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#ifndef MIDBRAIN_REFLEXES_HPP_
#define MIDBRAIN_REFLEXES_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace srs
{

class Reflexes
{
public:
	Reflexes(ros::NodeHandle nodeHandle);
	virtual ~Reflexes();

	void OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan );

private:

	double			m_chuckWidth_;

	double 			m_chuckHalfWidth;

	double 			m_yPaddedOffset;

	double 			m_safeDistance;

	ros::Subscriber	m_laserScanSubscriber;

	ros::Publisher	m_commandPublisher;

};

} /* namespace srs */

#endif /* MIDBRAIN_REFLEXES_HPP_ */

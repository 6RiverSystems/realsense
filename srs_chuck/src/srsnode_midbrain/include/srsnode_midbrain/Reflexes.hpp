/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MIDBRAIN_REFLEXES_HPP_
#define MIDBRAIN_REFLEXES_HPP_

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <srslib_framework/MsgOperationalState.h>

namespace srs
{

class Reflexes
{
public:
	Reflexes(ros::NodeHandle nodeHandle);
	virtual ~Reflexes();

private:

	void CreateSubscribers( );

	void CreatePublishers( );

	void OnOperationalStateChanged( const srslib_framework::MsgOperationalState::ConstPtr& operationalState );

	void OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity );

	void OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan );

	static constexpr auto OPERATIONAL_STATE_TOPIC = "/info/operational_state";

	static constexpr auto VELOCITY_TOPIC = "/internal/drivers/brainstem/cmd_velocity";

	static constexpr auto SCAN_TOPIC = "/camera/depth/scan";

	static constexpr auto EVENT_TOPIC = "/ll_event";

	ros::NodeHandle 						m_nodeHandle;

	uint32_t 								m_detectedObjectsLimit;

	double 									m_safetyFactor;

	double 									m_maxLinearVelocity;

	double 									m_maxAngularVelocity;

	double									m_chuckWidth_;

	double 									m_chuckHalfWidth;

	double 									m_yPaddedOffset;

	double 									m_minSafeDistance;

	double 									m_maxSafeDistance;

	srslib_framework::MsgOperationalState	m_operationalState;

	geometry_msgs::Twist					m_velocity;

	ros::Subscriber							m_operationalStateSubscriber;

	ros::Subscriber							m_laserScanSubscriber;

	ros::Subscriber							m_velocitySubscriber;

	ros::Publisher							m_commandPublisher;

};

} /* namespace srs */

#endif /* MIDBRAIN_REFLEXES_HPP_ */

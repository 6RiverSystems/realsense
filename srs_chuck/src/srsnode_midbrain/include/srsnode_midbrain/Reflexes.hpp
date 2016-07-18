/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MIDBRAIN_REFLEXES_HPP_
#define MIDBRAIN_REFLEXES_HPP_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <srsnode_midbrain/ObstacleDetector.hpp>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <srslib_framework/MsgOperationalState.h>
#include <dynamic_reconfigure/server.h>
#include <srsnode_midbrain/ReflexesConfig.h>

namespace srs
{

class Reflexes
{
public:
	Reflexes( ros::NodeHandle& nodeHandle );
	virtual ~Reflexes( );

// Configuration Options

	void Enable( bool enable );

	void SetObjectThreshold( uint32_t objectThreshold );

// Topic Callbacks

	void OnOperationalStateChanged( const srslib_framework::MsgOperationalState::ConstPtr& operationalState );

	void OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity );

	void OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan );

	void PublishDangerZone( ) const;

private:

    void onConfigChange(srsnode_midbrain::ReflexesConfig& config, uint32_t level);

    void OnObstacleDetected( );

	void CreateSubscribers( );
	void DestroySubscribers( );

	void CreatePublishers( );
	void DestroyPublishers( );

	static constexpr auto OPERATIONAL_STATE_TOPIC = "/info/operational_state";

	static constexpr auto VELOCITY_TOPIC = "/internal/drivers/brainstem/cmd_velocity";

	static constexpr auto SCAN_TOPIC = "/camera/scan";

	static constexpr auto DANGER_ZONE_TOPIC = "/internal/state/reflexes/danger_zone";

	static constexpr auto EVENT_TOPIC = "/ll_event";

	// TODO: Get footprint from robot model (lookup topic, load urdf, etc.)
	static constexpr auto ROBOT_WIDTH = 0.64f;

    dynamic_reconfigure::Server<srsnode_midbrain::ReflexesConfig> m_configServer;

	ros::NodeHandle&						m_nodeHandle;

	bool									m_enable;

	bool	 								m_sendHardStop;

	srslib_framework::MsgOperationalState	m_operationalState;

	ObstacleDetector 						m_obstacleDetector;

	ros::Subscriber							m_operationalStateSubscriber;

	ros::Subscriber							m_laserScanSubscriber;

	ros::Subscriber							m_velocitySubscriber;

	ros::Publisher							m_commandPublisher;

    ros::Publisher							m_dangerZonePublisher;

};

} /* namespace srs */

#endif /* MIDBRAIN_REFLEXES_HPP_ */

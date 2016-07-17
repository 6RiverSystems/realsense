/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tgmath.h>

namespace srs
{

Reflexes::Reflexes( ros::NodeHandle& nodeHandle ) :
	m_nodeHandle( nodeHandle ),
	m_obstacleDetector( 0.64f ),
	m_operationalStateSubscriber( ),
	m_laserScanSubscriber( ),
	m_velocitySubscriber( ),
	m_commandPublisher( )
{
	Enable( true );
}

Reflexes::~Reflexes( )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration Options
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Reflexes::Enable( bool enable )
{
	if( enable )
	{
		CreateSubscribers( );

		CreatePublishers( );
	}
	else
	{
		DestroySubscribers( );

		DestroyPublishers( );
	}
}

void Reflexes::SetObjectThreshold( uint32_t objectThreshold )
{
	m_obstacleDetector.SetObjectThreshold( objectThreshold );
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Topic Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Reflexes::OnOperationalStateChanged( const srslib_framework::MsgOperationalState::ConstPtr& operationalState )
{
//	m_obstacleDetector.Enable( );
}

void Reflexes::OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity )
{
	m_obstacleDetector.SetVelocity( velocity->linear.x, velocity->angular.z );
}

void Reflexes::OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan )
{
	m_obstacleDetector.ProcessScan( scan );
}

void Reflexes::onConfigChange(srsnode_midbrain::ReflexesConfig& config, uint32_t level)
{
	SetObjectThreshold( config.object_threshold );

	Enable( config.enable_obstacle_detection );

	m_sendHardStop = config.enable_hard_stop;
}

void Reflexes::CreateSubscribers( )
{
	m_operationalStateSubscriber = m_nodeHandle.subscribe<srslib_framework::MsgOperationalState>(
		OPERATIONAL_STATE_TOPIC, 10, std::bind( &Reflexes::OnOperationalStateChanged, this, std::placeholders::_1 ) );

	m_laserScanSubscriber = m_nodeHandle.subscribe<sensor_msgs::LaserScan>(
		SCAN_TOPIC, 10, std::bind( &Reflexes::OnLaserScan, this, std::placeholders::_1 ) );

	m_velocitySubscriber = m_nodeHandle.subscribe<geometry_msgs::Twist>( VELOCITY_TOPIC, 1,
	    std::bind( &Reflexes::OnChangeVelocity, this, std::placeholders::_1 ) );
}

void Reflexes::DestroySubscribers( )
{
	m_operationalStateSubscriber.shutdown( );

	m_laserScanSubscriber.shutdown( );

	m_velocitySubscriber.shutdown( );
}

void Reflexes::CreatePublishers( )
{
	m_commandPublisher = m_nodeHandle.advertise<std_msgs::String>( EVENT_TOPIC, 1, true );
}

void Reflexes::DestroyPublishers( )
{
	m_commandPublisher.shutdown( );
}

} /* namespace srs */

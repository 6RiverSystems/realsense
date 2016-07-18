/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Reflexes.hpp>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tgmath.h>
#include <boost/range/adaptor/reversed.hpp>

namespace srs
{

Reflexes::Reflexes( ros::NodeHandle& nodeHandle ) :
	m_nodeHandle( nodeHandle ),
	m_enable( false ),
	m_sendHardStop( false ),
	m_operationalState( ),
	m_obstacleDetector( ROBOT_WIDTH / 2.0f ),
	m_operationalStateSubscriber( ),
	m_laserScanSubscriber( ),
	m_velocitySubscriber( ),
	m_commandPublisher( )
{
	bool obstacleDetected = false;

	m_obstacleDetector.SetDetectionCallback( std::bind( &Reflexes::OnObstacleDetected, this ) );

	m_configServer.setCallback( boost::bind( &Reflexes::onConfigChange, this, _1, _2 ) );
}

Reflexes::~Reflexes( )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Configuration Options
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Reflexes::Enable( bool enable )
{
	if( m_enable != enable )
	{
		if( enable )
		{
			ROS_INFO_NAMED( "obstacle_detection", "Enabling Obstacle Detection" );

			CreateSubscribers( );

			CreatePublishers( );
		}
		else
		{
			ROS_INFO_NAMED( "obstacle_detection", "Disabling Obstacle Detection" );

			DestroySubscribers( );

			DestroyPublishers( );
		}

		m_enable = enable;
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
	ROS_INFO_STREAM_NAMED( "obstacle_detection", "OnOperationalStateChanged: " << operationalState->pause );

	m_operationalState = *operationalState;
}

void Reflexes::OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity )
{
	m_obstacleDetector.SetVelocity( velocity->linear.x, velocity->angular.z );
}

void Reflexes::OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan )
{
	m_obstacleDetector.ProcessScan( scan );
}

void Reflexes::PublishDangerZone( ) const
{
	geometry_msgs::PolygonStamped messageLanding;

	messageLanding.header.frame_id = "laser_frame";
	messageLanding.header.stamp = ros::Time::now( );

	std::vector<geometry_msgs::Point32> polygon;

	Polygon dangerZone = m_obstacleDetector.GetDangerZone( );

	for( const Point& point : dangerZone )
	{
		geometry_msgs::Point32 corner;
		corner.x = point.x;
		corner.y = point.y;
		corner.z = 0.0;

		polygon.push_back( corner );
	}

	messageLanding.polygon.points = polygon;

	m_dangerZonePublisher.publish( messageLanding );
}

void Reflexes::onConfigChange(srsnode_midbrain::ReflexesConfig& config, uint32_t level)
{
	ROS_INFO_STREAM_NAMED( "obstacle_detection", "Midbrain Configuration changed: \
		enable_obstacle_detection: " << config.enable_obstacle_detection <<
		", enable_hard_stop: " << config.enable_hard_stop <<
		", object_threshold: " << config.object_threshold );

	SetObjectThreshold( config.object_threshold );

	Enable( config.enable_obstacle_detection );

	m_sendHardStop = config.enable_hard_stop;
}

void Reflexes::OnObstacleDetected( )
{
	ROS_INFO_STREAM_NAMED( "obstacle_detection", "OnObstacleDetected: \
		m_operationalState.pause: " << m_operationalState.pause <<
		", m_enable: " << m_enable <<
		", m_sendHardStop: " << m_sendHardStop );

	// Only send a hard stop if we are paused
	if( !m_operationalState.pause &&
		m_enable &&
		m_sendHardStop )
	{
		// Send the hard stop
		std_msgs::String msg;
		msg.data = "STOP";

		m_commandPublisher.publish( msg );
	}
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

	m_dangerZonePublisher = m_nodeHandle.advertise<geometry_msgs::PolygonStamped>(DANGER_ZONE_TOPIC, 1);
}

void Reflexes::DestroyPublishers( )
{
	m_commandPublisher.shutdown( );
}

} /* namespace srs */

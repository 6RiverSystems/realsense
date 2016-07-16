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
	m_objectThreshold( 0 ),
	m_chuckWidth_( 0.619125 ),
	m_chuckHalfWidth( m_chuckWidth_ / 2.0f ),
	m_yPaddedOffset( m_chuckHalfWidth * 1.1 ),
	m_minSafeDistance( 0.35f ),
	m_maxSafeDistance( 0.5f ),
	m_operationalState( ),
	m_velocity( ),
	m_operationalStateSubscriber( ),
	m_laserScanSubscriber( ),
	m_velocitySubscriber( ),
	m_commandPublisher( )
{
	ROS_INFO( "Reflex: half width: %f, yOffset: %f, safeDistance: %f/%f",
		m_chuckHalfWidth, m_yPaddedOffset, m_minSafeDistance, m_maxSafeDistance );

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
	m_objectThreshold = objectThreshold;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Topic Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Reflexes::OnOperationalStateChanged( const srslib_framework::MsgOperationalState::ConstPtr& operationalState )
{
	m_operationalState = *operationalState;
}

void Reflexes::OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity )
{
	m_velocity = *velocity;
}

void Reflexes::OnLaserScan( const sensor_msgs::LaserScan::ConstPtr& scan )
{
	uint32_t numberOfObstacles = 0;

	uint32_t numberOfScans = scan->ranges.size( );

	// Calculated from actual measurments and polynomial regression fitting
	// https://docs.google.com/a/6river.com/spreadsheets/d/1wUPgpESlp-MVbnMGCmFDGH22qyO39D8kzvi7IPZ1Q1c/edit?usp=sharing
	// http://www.xuru.org/rt/PR.asp#CopyPaste
	double safeDistance = 0.3518981019f * pow( m_velocity.linear.x, 2 ) - 0.03806193806f * m_velocity.linear.x + 0.01804195804f;

	std::string strPoints;

	char point[255] = { '\0' };
	sprintf( point, "Safe distance (velocity: %.2f): %.2f: ", m_velocity.linear.x, safeDistance );
	strPoints += point;

	// Apply min/max to the safe distance
	safeDistance = std::min( safeDistance, m_maxSafeDistance );
	safeDistance = std::max( safeDistance, m_minSafeDistance );

	sprintf( point, "Safe distance clamped (velocity: %.2f): %.2f: ", m_velocity.linear.x, safeDistance );
	strPoints += point;

	for( int x = 0; x < numberOfScans; x++ )
	{
		double angle = ((double)x * scan->angle_increment) + scan->angle_min;

		double scanDistance = scan->ranges[x];

		if( !std::isinf( scanDistance ) )
		{
			//            y Offset
			//             ------
			//             \    |
			//              \   |
			// scan distance \  | Distance from bot
			//                \0|
			//                 \|

			double yOffset = scanDistance * tan( angle );
			double distanceFromBot = yOffset / sin( angle );

			sprintf( point, "%.2f, ", fabs( distanceFromBot ));
			strPoints += point;

			if( fabs( distanceFromBot ) < safeDistance )
			{
				if( fabs( yOffset ) < m_yPaddedOffset )
				{
					numberOfObstacles++;
				}
			}
		}
	}

	ROS_DEBUG_THROTTLE_NAMED( 0.3, "reflexes", "linear vel: %0.2f, safeDistance: %.02f, Obstacles detected: %d, points: %s",
		m_velocity.linear.x, safeDistance, numberOfObstacles, strPoints.c_str( ) );

	if( numberOfObstacles > m_objectThreshold )
	{
		// Only send a hard stop if we are paused
		if( !m_operationalState.pause )
		{

			if( m_sendHardStop )
			{
				// Send the hard stop
				std_msgs::String msg;
				msg.data = "STOP";

				m_commandPublisher.publish( msg );
			}
		}
	}
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

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStem.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/TwistStamped.h>
#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/platform/Thread.hpp>

bool approximatively_equal(double x, double y, int ulp)
{
   return fabs(x-y) <= ulp*DBL_EPSILON*std::max(fabs(x), fabs(y));
}

namespace srs
{

BrainStem::BrainStem( const std::string& strSerialPort ) :
	m_rosNodeHandle( ),
    m_VelocitySubscriber( m_rosNodeHandle.subscribe<geometry_msgs::Twist>(
        "/internal/drivers/brainstem/cmd_velocity", 100,
        std::bind( &BrainStem::OnChangeVelocity, this, std::placeholders::_1 ) ) ),
    m_OdometryRawPublisher( m_rosNodeHandle.advertise<geometry_msgs::TwistStamped>(
        "/internal/sensors/odometry/raw", 1000 ) ),
    m_ConnectedPublisher( m_rosNodeHandle.advertise<std_msgs::Bool>(
        "/internal/drivers/brainstem/connected", 1, true) ),
	m_llcmdSubscriber( m_rosNodeHandle.subscribe<std_msgs::String>( "/cmd_ll", 1000,
		std::bind( &BrainStem::OnRosCallback, this, std::placeholders::_1 ) ) ),
	m_llEventPublisher( m_rosNodeHandle.advertise<std_msgs::String>( "/ll_event", 50 ) ),
	m_pSerialIO( new SerialIO( ) ),
	m_messageProcessor( m_pSerialIO ),
	m_dwLastOdomTime( 0 ),
	m_rosOdomTime( )
{
	m_messageProcessor.SetConnectionChangedCallback( std::bind( &BrainStem::OnConnectionChanged, this, std::placeholders::_1 ) );

	m_messageProcessor.SetArrivedCallback( std::bind( &BrainStem::OnArrived, this ) );

	m_messageProcessor.SetButtonCallback( std::bind( &BrainStem::OnButtonEvent, this, std::placeholders::_1 ) );

	m_messageProcessor.SetOdometryCallback(
		std::bind( &BrainStem::OnOdometryChanged, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 ) );

	std::shared_ptr<SerialIO> pSerialIO = std::dynamic_pointer_cast<SerialIO>( m_pSerialIO );

	pSerialIO->EnableCRC( true );
	pSerialIO->SetTerminatingCharacter( '\n' );
	pSerialIO->SetEscapeCharacter( '\\' );

	auto processMessage = [&]( std::vector<char> buffer )
	{
		ExecuteInRosThread( std::bind( &BrainStemMessageProcessor::ProcessBrainStemMessage, &m_messageProcessor,
				buffer ) );
	};

	auto connectionChanged = [&]( bool bIsConnected )
	{
		ExecuteInRosThread( std::bind( &BrainStem::OnConnectionChanged, this,
				bIsConnected ) );
	};

	pSerialIO->Open( strSerialPort.c_str( ), connectionChanged, processMessage );
}

BrainStem::~BrainStem( )
{

}

void BrainStem::Run( )
{
	ros::Rate refreshRate( REFRESH_RATE_HZ );

	ros::spin( );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Callbacks
////////////////////////////////////////////////////////////////////////////////////////////////////

void BrainStem::OnConnectionChanged( bool bIsConnected )
{
	std_msgs::Bool msg;
	msg.data = bIsConnected;

	m_ConnectedPublisher.publish( msg );

	m_rosOdomTime = ros::Time( );

//	// Get the hardware information
//	GetHardwareInformation( );
//
//	// Get the operational state
//	GetOperationalState( );
}

void BrainStem::OnArrived( )
{
	std_msgs::String msg;
	msg.data = "ARRIVED";

	ROS_DEBUG_NAMED( "Brainstem", "%s", msg.data.c_str( ) );

	m_llEventPublisher.publish( msg );
}

void BrainStem::OnButtonEvent( LED_ENTITIES eButtonId )
{
	std::string strEntity = m_messageProcessor.GetButtonName( eButtonId );

	if( strEntity.length( ) )
	{
		std_msgs::String msg;

		std::stringstream ss;
		ss << "UI " << strEntity;
		msg.data = ss.str();

		ROS_DEBUG_NAMED( "Brainstem", "%s", msg.data.c_str( ) );

		m_llEventPublisher.publish( msg );
	}
	else
	{
		ROS_ERROR_NAMED( "Brainstem", "Unknown button entity %d", eButtonId );
	}
}

void BrainStem::OnOdometryChanged( uint32_t dwTimeStamp, float fLinearVelocity, float fAngularVelocity )
{
	ros::Time currentTime = ros::Time::now( );

	static ros::Time sLastTime = currentTime;

	bool bInvalidTime = ( currentTime.toSec( ) - m_rosOdomTime.toSec( ) ) > 0.15;

	if( !m_rosOdomTime.isZero( ) &&
		bInvalidTime )
	{
		ROS_ERROR_NAMED( "Brainstem", "Timestamp out of range and resynced (possible communication problem) (diff: %f): odom: %f, ros: %f",
			currentTime.toSec( ) - m_rosOdomTime.toSec( ), m_rosOdomTime.toSec( ), currentTime.toSec( ) );
	}

	if( bInvalidTime ||
		( approximatively_equal( fLinearVelocity, 0.0f, 0.00001 ) &&
		  approximatively_equal( fAngularVelocity, 0.0f, 0.00001 ) ) )
	{
		// Reset our time basis to account for any drift
		m_rosOdomTime = currentTime - ros::Duration( 0, 1200000 );
	}
	else
	{
		double dfOdomTimeDelta = (double)(dwTimeStamp - m_dwLastOdomTime) / 1000.0f;

		ROS_DEBUG_NAMED( "Brainstem", "Odometry (%f): %f, %f",
			dfOdomTimeDelta, fLinearVelocity, fAngularVelocity );

		// Base our time on the realtime clock (brain_stem) since our clock does not match odom info
		// and our loop is not realtime
		m_rosOdomTime += ros::Duration( dfOdomTimeDelta );
	}

	geometry_msgs::TwistStamped odometry;
	odometry.header.stamp = m_rosOdomTime;
	odometry.twist.linear.x = fLinearVelocity;
	odometry.twist.angular.z = fAngularVelocity;

	m_OdometryRawPublisher.publish( odometry );

	double dfClockDiff = currentTime.toSec( ) - m_rosOdomTime.toSec( );

	if( dfClockDiff > 0.05 )
	{
		ROS_ERROR_NAMED( "Brainstem", "Odometry clock drift: %f", dfClockDiff );
	}

	m_dwLastOdomTime = dwTimeStamp;

	sLastTime = currentTime;
}

void BrainStem::GetOperationalState( )
{
	m_messageProcessor.GetOperationalState( );
}

void BrainStem::GetHardwareInformation( )
{
	m_messageProcessor.GetHardwareInformation( );
}

void BrainStem::OnChangeVelocity( const geometry_msgs::Twist::ConstPtr& velocity )
{
	m_messageProcessor.SetVelocity( velocity->linear.x, velocity->angular.z );
}

void BrainStem::OnRosCallback( const std_msgs::String::ConstPtr& msg )
{
	m_messageProcessor.ProcessRosMessage( msg->data );
}

}// namespace srs

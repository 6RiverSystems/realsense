/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStem.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <srslib_framework/HardwareInfo.h>
#include <srslib_framework/OperationalState.h>
#include <geometry_msgs/TwistStamped.h>
#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <bitset>

bool approximatively_equal(double x, double y, int ulp)
{
   return fabs(x-y) <= ulp*DBL_EPSILON*std::max(fabs(x), fabs(y));
}

namespace srs
{

static constexpr auto REFRESH_RATE_HZ = 100;

static constexpr auto HARDWARE_INFO_TOPIC = "/info/hardware";

static constexpr auto OPERATIONAL_STATE_TOPIC = "/info/operational_state";

static constexpr auto VOLTAGE_TOPIC = "/info/voltage";

static constexpr auto CONNECTED_TOPIC = "/internal/drivers/brainstem/connected";

static constexpr auto VELOCITY_TOPIC = "/internal/drivers/brainstem/cmd_velocity";

static constexpr auto ODOMETRY_TOPIC = "/internal/sensors/odometry/raw";

static constexpr auto PING_TOPIC = "/internal/state/ping";

// TODO: Remove/Replace with proper messages
static constexpr auto COMMAND_TOPIC = "/cmd_ll";

static constexpr auto EVENT_TOPIC = "/ll_event";

BrainStem::BrainStem( const std::string& strSerialPort ) :
	m_rosNodeHandle( ),
	m_pingSubscriber( ),
    m_velocitySubscriber( ),
    m_odometryRawPublisher( ),
    m_connectedPublisher( ),
	m_llcmdSubscriber( ),
	m_llEventPublisher( ),
	m_hardwareInfoPublisher( ),
	m_operationalStatePublisher( ),
	m_voltagePublisher( ),
	m_pSerialIO( new SerialIO( "brainstem" ) ),
	m_messageProcessor( m_pSerialIO ),
	m_dwLastOdomTime( 0 ),
	m_rosOdomTime( )
{
	CreateSubscribers( );

	CreatePublishers( );

	SetupCallbacks( );

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

	m_connectedPublisher.publish( msg );

	m_rosOdomTime = ros::Time( );

	// Get the hardware information
	GetHardwareInformation( );

	// Get the operational state
	GetOperationalState( );
}

void BrainStem::OnButtonEvent( LED_ENTITIES eButtonId )
{
	std::string strEntity = m_messageProcessor.GetButtonName( eButtonId );

	if( strEntity.length( ) )
	{
		std_msgs::String msg;

		std::stringstream ss;
		ss << "UI " << strEntity;
		msg.data = ss.str( );

		ROS_DEBUG( "%s", msg.data.c_str( ) );

		m_llEventPublisher.publish( msg );
	}
	else
	{
		ROS_ERROR( "Unknown button entity %d", eButtonId );
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
		ROS_ERROR( "Timestamp out of range and resynced (possible communication problem) (diff: %f): odom: %f, ros: %f",
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

		ROS_DEBUG_NAMED( "odometry", "Odometry (%f): %f, %f", dfOdomTimeDelta, fLinearVelocity, fAngularVelocity );

		// Base our time on the realtime clock (brain_stem) since our clock does not match odom info
		// and our loop is not realtime
		m_rosOdomTime += ros::Duration( dfOdomTimeDelta );
	}

	geometry_msgs::TwistStamped odometry;
	odometry.header.stamp = m_rosOdomTime;
	odometry.twist.linear.x = fLinearVelocity;
	odometry.twist.angular.z = fAngularVelocity;

	m_odometryRawPublisher.publish( odometry );

	double dfClockDiff = currentTime.toSec( ) - m_rosOdomTime.toSec( );

	if( dfClockDiff > 0.05 )
	{
		ROS_ERROR( "Odometry clock drift: %f", dfClockDiff );
	}

	m_dwLastOdomTime = dwTimeStamp;

	sLastTime = currentTime;
}

void BrainStem::OnHardwareInfo( uint32_t uniqueId[4], uint8_t bodyType, uint32_t configuration,
	uint32_t lifetimeHours, uint32_t lifetimeMeters, uint32_t batteryHours,
	uint32_t wheelMeters, const std::string& strBrainstemVersion )
{
	char pszGuid[255] = { '\0' };
	sprintf( pszGuid, "%08X-%08X-%08X-%08X", uniqueId[0], uniqueId[1], uniqueId[2], uniqueId[3] );

	std::string strName( getenv( "ROBOT_NAME" ) );

	ROS_INFO_STREAM( "Hardware Info => name: " << strName << ", id: " << pszGuid << ", bodyType: " << bodyType <<
		", configuration: " << configuration << ", lifetimeHours: " << lifetimeHours <<
		", lifetimeMeters:"  << lifetimeMeters << ", batteryHours: " << batteryHours <<
		", wheelMeters: " << wheelMeters << ", Brainstem Version: " << strBrainstemVersion );

	srslib_framework::HardwareInfo msg;;
	msg.name = strName;
	msg.uniqueId = pszGuid;
	msg.bodyType = bodyType;
	msg.configuration = configuration;
	msg.lifetimeHours = lifetimeHours;
	msg.lifetimeMeters = lifetimeMeters;
	msg.batteryHours = batteryHours;
	msg.wheelMeters = wheelMeters;
	msg.brainstemVersion = strBrainstemVersion;

	m_hardwareInfoPublisher.publish( msg );
}

void BrainStem::OnOperationalStateChanged( uint32_t upTime, const MOTION_STATUS_DATA& motionStatus,
	const FAILURE_STATUS_DATA& failureStatus )
{
	std::ostringstream stream;

	stream << "Operational State => uptime:" << upTime <<
		", frontEStop: " << motionStatus.frontEStop <<
		", backEStop: " << motionStatus.backEStop <<
		", wirelessEStop: " << motionStatus.wirelessEStop <<
		", bumpSensor: " << motionStatus.bumpSensor <<
		", pause: " << motionStatus.pause <<
		", hardStop: " << motionStatus.hardStop <<
		", safetyProcessorFailure: " << failureStatus.safetyProcessorFailure <<
		", brainstemFailure: " << failureStatus.brainstemFailure <<
		", brainTimeoutFailure: " << failureStatus.brainTimeoutFailure <<
		", rightMotorFailure: " << failureStatus.rightMotorFailure <<
		", leftMotorFailure: " << failureStatus.leftMotorFailure <<
		std::endl;

	std::string strData =  stream.str( );

	ROS_INFO_STREAM( strData );

	srslib_framework::OperationalState msg;
	msg.frontEStop = motionStatus.frontEStop;
	msg.backEStop = motionStatus.backEStop;
	msg.wirelessEStop = motionStatus.wirelessEStop;
	msg.bumpSensor = motionStatus.bumpSensor;
	msg.pause = motionStatus.pause;
	msg.hardStop = motionStatus.hardStop;
	msg.safetyProcessorFailure = failureStatus.safetyProcessorFailure;
	msg.brainstemFailure = failureStatus.brainstemFailure;
	msg.brainTimeoutFailure = failureStatus.brainTimeoutFailure;
	msg.rightMotorFailure = failureStatus.rightMotorFailure;
	msg.leftMotorFailure = failureStatus.leftMotorFailure;

	m_operationalStatePublisher.publish( msg );
}

void BrainStem::OnVoltageChanged( float fVoltage )
{
	ROS_INFO_STREAM( "Voltage => " << fVoltage );

	std_msgs::Float32 msg;
	msg.data = fVoltage;

	m_voltagePublisher.publish( msg );
}

void BrainStem::CreateSubscribers( )
{
	m_pingSubscriber = m_rosNodeHandle.subscribe<std_msgs::Bool>( PING_TOPIC, 10,
	    std::bind( &BrainStem::OnPing, this ) );

	m_velocitySubscriber = m_rosNodeHandle.subscribe<geometry_msgs::Twist>( VELOCITY_TOPIC, 10,
	    std::bind( &BrainStem::OnChangeVelocity, this, std::placeholders::_1 ) );

	m_llcmdSubscriber = m_rosNodeHandle.subscribe<std_msgs::String>( COMMAND_TOPIC, 100,
	    std::bind( &BrainStem::OnRosCallback, this, std::placeholders::_1 ) );
}

void BrainStem::CreatePublishers( )
{
	m_connectedPublisher = m_rosNodeHandle.advertise<std_msgs::Bool>( CONNECTED_TOPIC, 1, true );

	m_hardwareInfoPublisher = m_rosNodeHandle.advertise<srslib_framework::HardwareInfo>( HARDWARE_INFO_TOPIC, 1, true );

	m_operationalStatePublisher = m_rosNodeHandle.advertise<srslib_framework::OperationalState>( OPERATIONAL_STATE_TOPIC, 1, true );

	m_voltagePublisher = m_rosNodeHandle.advertise<std_msgs::Float32>( VOLTAGE_TOPIC, 1, true );

	m_odometryRawPublisher = m_rosNodeHandle.advertise<geometry_msgs::TwistStamped>( ODOMETRY_TOPIC, 100 );

	m_llEventPublisher = m_rosNodeHandle.advertise<std_msgs::String>( EVENT_TOPIC, 100 );
}

void BrainStem::SetupCallbacks( )
{
	m_messageProcessor.SetConnectionChangedCallback( std::bind( &BrainStem::OnConnectionChanged,
		this, std::placeholders::_1 ) );

	m_messageProcessor.SetButtonCallback( std::bind( &BrainStem::OnButtonEvent, this, std::placeholders::_1 ) );

	m_messageProcessor.SetOdometryCallback( std::bind( &BrainStem::OnOdometryChanged, this,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 ) );

	m_messageProcessor.SetHardwareInfoCallback( std::bind( &BrainStem::OnHardwareInfo, this,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
		std::placeholders::_5, std::placeholders::_6, std::placeholders::_7, std::placeholders::_8 ) );

	m_messageProcessor.SetOperationalStateCallback( std::bind( &BrainStem::OnOperationalStateChanged, this,
		std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 ) );

	m_messageProcessor.SetVoltageCallback( std::bind( &BrainStem::OnVoltageChanged, this,
		std::placeholders::_1 ) );
}

void BrainStem::GetOperationalState( )
{
	m_messageProcessor.GetOperationalState( );
}

void BrainStem::GetHardwareInformation( )
{
	m_messageProcessor.GetHardwareInformation( );
}

void BrainStem::OnPing( )
{
	m_messageProcessor.SendPing( );
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

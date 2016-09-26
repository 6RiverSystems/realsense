/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <BrainStem.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <srslib_framework/MsgHardwareInfo.h>
#include <srslib_framework/MsgOperationalState.h>
#include <geometry_msgs/TwistStamped.h>
#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <bitset>

#include <srslib_framework/math/BasicMath.hpp>

namespace srs
{

BrainStem::BrainStem( const std::string& strSerialPort ) :
	m_rosNodeHandle( ),
	m_pingSubscriber( ),
    m_velocitySubscriber( ),
    m_connectedPublisher( ),
	m_llcmdSubscriber( ),
	m_llEventPublisher( ),
	m_operationalStatePublisher( ),
	m_voltagePublisher( ),
	m_pSerialIO( new SerialIO( "brainstem" ) ),
	m_messageProcessor( m_pSerialIO )
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
        ExecuteInRosThread( std::bind( &BrainStemMessageProcessor::processHardwareMessage,
            &m_messageProcessor, buffer));
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

	srslib_framework::MsgOperationalState msg;
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

	m_operationalStatePublisher = m_rosNodeHandle.advertise<srslib_framework::MsgOperationalState>(
	    OPERATIONAL_STATE_TOPIC, 1, true );

	m_voltagePublisher = m_rosNodeHandle.advertise<std_msgs::Float32>( VOLTAGE_TOPIC, 1, true );

	m_llEventPublisher = m_rosNodeHandle.advertise<std_msgs::String>( EVENT_TOPIC, 100 );
}

void BrainStem::SetupCallbacks( )
{
	m_messageProcessor.SetConnectionChangedCallback( std::bind( &BrainStem::OnConnectionChanged,
		this, std::placeholders::_1 ) );

	m_messageProcessor.SetButtonCallback( std::bind( &BrainStem::OnButtonEvent, this, std::placeholders::_1 ) );

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

void BrainStem::OnRosCallback(const std_msgs::String::ConstPtr& msg)
{
    m_messageProcessor.processRosMessage(msg->data);
}

}// namespace srs

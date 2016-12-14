/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include "../include/srsdrv_brainstem/BrainStem.hpp"

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <srslib_framework/MsgHardwareInfo.h>
#include <srslib_framework/MsgOperationalState.h>
#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <bitset>

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/ros/topics/ChuckTopics.hpp>

namespace srs
{

BrainStem::BrainStem( const std::string& strSerialPort ) :
	m_pSerialIO( new SerialIO( "brainstem" ) ),
	m_messageProcessor( m_pSerialIO ),
	brainstemFaultTimer_(),
	nodeHandle_("~")
{
	OnConnectionChanged( false );

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


    // Check for hardware faults
	brainstemFaultTimer_ = nodeHandle_.createTimer(ros::Duration(1.0f / REFRESH_RATE_HZ),
        boost::bind(&BrainStemMessageProcessor::checkForBrainstemFaultTimer,
        	&m_messageProcessor, _1));
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
	m_messageProcessor.setConnected(bIsConnected);

	if( !bIsConnected )
	{
		m_brainstemEmulator.reset( new BrainStemEmulator( ) );
	}
	else
	{
		m_brainstemEmulator.reset( );
	}
}

}// namespace srs

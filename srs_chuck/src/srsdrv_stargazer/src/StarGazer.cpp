/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <StarGazer.h>
#include <srslib_framework/Aps.h>
#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/platform/Thread.hpp>

namespace srs
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

// TODO: Pass in serial port and #define
////////////////////////////////////////////////////////////////////////////////////////////////////

StarGazer::StarGazer( const std::string& strSerialPort, const std::string& strApsTopic ) :
	m_rosNodeHandle( ),
	m_rosApsPublisher( m_rosNodeHandle.advertise<srslib_framework::Aps>( strApsTopic, 1000 ) ),
	m_pSerialIO( new SerialIO( ) ),
	m_messageProcessor( m_pSerialIO )
{
	m_messageProcessor.SetOdometryCallback(
		std::bind( &StarGazer::OdometryCallback, this, std::placeholders::_1, std::placeholders::_2,
			std::placeholders::_3, std::placeholders::_4, std::placeholders::_5 ) );

	m_messageProcessor.SetReadCallback(
		std::bind( &StarGazer::ReadCallback, this, std::placeholders::_1, std::placeholders::_2 ) );

	std::shared_ptr<SerialIO> pSerialIO = std::dynamic_pointer_cast<SerialIO>( m_pSerialIO );

	pSerialIO->SetLeadingCharacter( STARGAZER_STX );
	pSerialIO->SetTerminatingCharacter( STARGAZER_RTX );
	pSerialIO->SetFirstByteDelay( std::chrono::microseconds( 30000 ) );
	pSerialIO->SetByteDelay( std::chrono::microseconds( 2000 ) );

	auto processMessage = [&]( std::vector<char> buffer )
		{
			ExecuteInRosThread( std::bind( &StarGazerMessageProcessor::ProcessStarGazerMessage, &m_messageProcessor,
				buffer ) );
		};

	auto connectionChanged = [&]( bool bIsConnected )
	{
		ExecuteInRosThread( std::bind( &StarGazer::OnConnectionChanged, this,
				bIsConnected ) );
	};

	pSerialIO->Open( strSerialPort.c_str( ), connectionChanged, processMessage );
}

StarGazer::~StarGazer( )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void StarGazer::Run( )
{
	ros::Rate refreshRate( REFRESH_RATE_HZ );

	while( ros::ok( ) )
	{
		ros::spinOnce( );

		PumpMessageProcessor( );

		refreshRate.sleep( );
	}
}


void StarGazer::OnConnectionChanged( bool bIsConnected )
{
	if( bIsConnected )
	{
		Configure( );

		Start( );
	}
}

void StarGazer::HardReset( )
{
	Stop( );

	m_messageProcessor.HardReset( );
}

void StarGazer::Configure( )
{
	m_messageProcessor.ResetState( );

	Stop( );

	m_messageProcessor.GetVersion( );

	m_messageProcessor.SetMarkType( STAR_GAZER_LANDMARK_TYPES::HLD3L );

	m_messageProcessor.SetEnd( );
}

void StarGazer::SetFixedHeight( int height_mm )
{
	Stop( );

	m_messageProcessor.SetMarkHeight( height_mm );
	m_messageProcessor.HightFix( true );
	m_messageProcessor.SetEnd( );
}

void StarGazer::SetVariableHeight( )
{
	Stop( );

	m_messageProcessor.HightFix( false );
	m_messageProcessor.SetEnd( );
}

void StarGazer::AutoCalculateHeight( )
{
	Stop( );

	m_messageProcessor.HeightCalc( );
}

void StarGazer::Start( )
{
	m_messageProcessor.CalcStart( );
}

void StarGazer::Stop( )
{
	m_messageProcessor.CalcStop( );
}

void StarGazer::PumpMessageProcessor( )
{
	m_messageProcessor.PumpMessageProcessor( );
}

void StarGazer::ReadCallback( std::string strType, std::string strValue )
{
	ROS_DEBUG_STREAM_NAMED( "StarGazer", strType << " = " << strValue );
}

void StarGazer::OdometryCallback( int nTagId, float fX, float fY, float fZ, float fAngle )
{
	srslib_framework::Aps msg;

	msg.tagId = nTagId;
	msg.x = fX;
	msg.y = fY;
	msg.z = fZ;
	msg.yaw = fAngle;

	m_rosApsPublisher.publish( msg );

	ROS_DEBUG_NAMED( "StarGazer", "Tag: %04i (%2.2f, %2.2f, %2.2f) %2.2f deg", nTagId, fX, fY, fZ, fAngle );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs

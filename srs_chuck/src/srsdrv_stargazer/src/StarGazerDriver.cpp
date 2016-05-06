/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include "StarGazerDriver.h"
#include <iostream>
#include <ros/ros.h>

namespace srs
{

StarGazerDriver::StarGazerDriver( const std::string& strSerialPort ) :
	m_bStarted( true ),
	m_messageProcessor( strSerialPort )
{
	m_messageProcessor.SetReadCallback(
		std::bind( &StarGazerDriver::ReadCallback, this, std::placeholders::_1, std::placeholders::_2 ) );
}

StarGazerDriver::~StarGazerDriver( )
{

}

void StarGazerDriver::SetOdometryCallback( OdometryCallbackFn callback )
{
	m_messageProcessor.SetOdometryCallback( callback );
}

void StarGazerDriver::HardReset( )
{
	Stop( );

	m_messageProcessor.HardReset( );
}

void StarGazerDriver::Configure( )
{
	Stop( );

	m_messageProcessor.GetVersion( );

	m_messageProcessor.SetMarkType( STAR_GAZER_LANDMARK_TYPES::HLD3L );

	m_messageProcessor.SetEnd( );
}

void StarGazerDriver::AutoCalculateHeight( )
{
	Stop( );

	m_messageProcessor.HeightCalc( );
}

void StarGazerDriver::Start( )
{
	m_messageProcessor.CalcStart( );
}

void StarGazerDriver::Stop( )
{
	m_messageProcessor.CalcStop( );
}

void StarGazerDriver::PumpMessageProcessor( )
{
	m_messageProcessor.PumpMessageProcessor( );
}

void StarGazerDriver::ReadCallback( std::string strType, std::string strValue )
{
	ROS_DEBUG_STREAM_NAMED( "StarGazer", strType << " = " << strValue );
}

} /* namespace srs */

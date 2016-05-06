/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <thread>
#include <chrono>
#include <iostream>
#include <ros/ros.h>
#include "StarGazerMessage.h"
#include "StarGazerSerialIO.h"

namespace srs
{

StarGazerSerialIO::StarGazerSerialIO( ) :
	SerialIO( ),
	m_interByteDelay( std::chrono::microseconds( 30000 ) )
{

}

StarGazerSerialIO::~StarGazerSerialIO( )
{

}

//void StarGazerSerialIO::OnWriteComplete( const boost::system::error_code& error, std::size_t size )
//{
////	ROS_DEBUG_STREAM_NAMED( "BrainStemSerialIO", "Write: " <<
////		ToHex( std::vector<char>( m_writeData.begin( ), m_writeData.end( ) + size ) ) );
//
//	std::vector<char>( m_writeData.begin( ) + size, m_writeData.end( ) ).swap( m_writeData );
//
//	if( m_writeData.begin( ) != m_writeData.end( ) )
//	{
//		std::this_thread::sleep_for( m_interByteDelay );
//		m_SerialPort.async_write_some( boost::asio::buffer( m_writeData.data( ), 1 ),
//			std::bind( &StarGazerSerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
//
//		// TOOD: Move these to a #define
//
//		// We have already sent out a character, so we only need to wait for 100uSec
//		m_interByteDelay = std::chrono::microseconds( 5000 );
//	}
//	else
//	{
//		m_bIsWriting = false;
//
//		// We are going to wait for an unknown amount of time, so lets wait the full 30mSec when a new byte is sent out
//		m_interByteDelay = std::chrono::microseconds( 30000 );
//
//	}
//}

//void StarGazerSerialIO::OnReadComplete( const boost::system::error_code& error, std::size_t size )
//{
//	if( !error )
//	{
//		// Combine buffers
//		m_readData.insert( m_readData.end( ), m_ReadBuffer.begin( ), m_ReadBuffer.begin( ) + size );
//
//		auto msgStart = std::find( m_readData.begin( ), m_readData.end( ), StarGazer_STX );
//		auto msgEnd = std::find( msgStart, m_readData.end( ), StarGazer_RTX );
//
//		// While we found a message, process it
//		while( msgEnd != m_readData.end( ) )
//		{
//			if( m_readCallback )
//			{
//				// Return the whole message
//				std::vector<char> msgBuffer( msgStart, msgEnd );
//				m_readCallback( msgBuffer );
//			}
//			else
//			{
//				ROS_ERROR_NAMED( "StarGazerSerialIO", "Serial port data read but no callback specified!\n" );
//			}
//
//			// Remove the consumed message
//			m_readData.erase( m_readData.begin( ), msgEnd );
//
//			msgStart = std::find( m_readData.begin( ), m_readData.end( ), StarGazer_STX );
//			msgEnd = std::find( msgStart, m_readData.end( ), StarGazer_RTX );
//		}
//	}
//}

} /* namespace srs */

/*
 * StarGazerSerialIO.cpp
 *
 *  Created on: May 2, 2016
 *      Author: cacioppo
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
	m_Thread( ),
	m_oWork( m_IOService ),
	m_SerialPort( m_IOService ),
	m_interByteDelay( std::chrono::microseconds( 30000 ) ),
	m_bIsWriting( false ),
	m_ReadBuffer( 1024 ),
	m_writeData( ),
	m_readData( ),
	m_readCallback( )
{
	// Spin up the thread
	m_Thread.reset( new std::thread( [&]()
	{
		m_IOService.run( );
	} ) );
}

StarGazerSerialIO::~StarGazerSerialIO( )
{
	Close( );

	// Stop the service (and the thread)
	m_IOService.stop( );

	// Clean up the thread
	m_Thread->join( );
}

void StarGazerSerialIO::Open( const char* pszName, std::function<void( std::vector<char> )> readCallback )
{
	m_readCallback = readCallback;

	try
	{
		m_SerialPort.open( pszName );

		// Setup serial port for 8/N/1 operation @ 115.2kHz
		m_SerialPort.set_option( boost::asio::serial_port::baud_rate( 115200 ) );
		m_SerialPort.set_option(
			boost::asio::serial_port::flow_control( boost::asio::serial_port::flow_control::none ) );
		m_SerialPort.set_option( boost::asio::serial_port::character_size( 8 ) );
		m_SerialPort.set_option( boost::asio::serial_port::parity( boost::asio::serial_port::parity::none ) );
		m_SerialPort.set_option( boost::asio::serial_port::stop_bits( boost::asio::serial_port::stop_bits::one ) );

		StartAsyncRead( );
	}
	catch( const std::exception& )
	{
		ROS_ERROR_NAMED( "StarGazerSerialIO", "Can not open serial port %s\n", pszName );
	}
}

bool StarGazerSerialIO::IsOpen( ) const
{
	return m_SerialPort.is_open( );
}

void StarGazerSerialIO::Close( )
{
	m_SerialPort.close( );
}

void StarGazerSerialIO::Write( const std::vector<char>& buffer )
{
	if( IsOpen( ) )
	{
		// Do all data operations in serial processing thread
		m_IOService.post( std::bind( &StarGazerSerialIO::WriteInSerialThread, this, buffer ) );
	}
	else
	{
		// Throw error
		throw std::runtime_error( "port not open" );
	}
}

void StarGazerSerialIO::WriteInSerialThread( std::vector<char> buffer )
{
	// Add payload data
	m_writeData.insert( m_writeData.end( ), buffer.begin( ), buffer.end( ) );

	// If we are already trying to write, then wait until we recieve the callback
	if( !m_bIsWriting )
	{
		m_bIsWriting = true;

		// We only write one byte at a time because the stupid sensor needs a delay in between each byte (and 30ms after the first byte)
		m_SerialPort.async_write_some( boost::asio::buffer( m_writeData.data( ), 1 ),
			std::bind( &StarGazerSerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
	}
}

void StarGazerSerialIO::StartAsyncRead( )
{
	m_SerialPort.async_read_some( boost::asio::buffer( m_ReadBuffer, m_ReadBuffer.size( ) ),
		std::bind( &StarGazerSerialIO::OnReadComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
}

void StarGazerSerialIO::OnWriteComplete( const boost::system::error_code& error, std::size_t size )
{
//	ROS_DEBUG_STREAM_NAMED( "SerialIO", "Write: " <<
//		ToHex( std::vector<char>( m_writeData.begin( ), m_writeData.end( ) + size ) ) );
	
	std::vector<char>( m_writeData.begin( ) + size, m_writeData.end( ) ).swap( m_writeData );

	if( m_writeData.begin( ) != m_writeData.end( ) )
	{
		std::this_thread::sleep_for( m_interByteDelay );
		m_SerialPort.async_write_some( boost::asio::buffer( m_writeData.data( ), 1 ),
			std::bind( &StarGazerSerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );

		// We have already sent out a character, so we only need to wait for 100uSec
		m_interByteDelay = std::chrono::microseconds( 5000 );
	}
	else
	{
		m_bIsWriting = false;

		// We are going to wait for an unknown amount of time, so lets wait the full 30mSec when a new byte is sent out
		m_interByteDelay = std::chrono::microseconds( 30000 );

	}
}

void StarGazerSerialIO::OnReadComplete( const boost::system::error_code& error, std::size_t size )
{
	if( !error )
	{
		// Combine buffers
		m_readData.insert( m_readData.end( ), m_ReadBuffer.begin( ), m_ReadBuffer.begin( ) + size );

		auto msgStart = std::find( m_readData.begin( ), m_readData.end( ), STARGAZER_STX );
		auto msgEnd = std::find( msgStart, m_readData.end( ), STARGAZER_RTX );

		// While we found a message, process it
		while( msgEnd != m_readData.end( ) )
		{
			if( m_readCallback )
			{
				// Return the whole message
				std::vector<char> msgBuffer( msgStart, msgEnd );
				m_readCallback( msgBuffer );
			}
			else
			{
				ROS_ERROR_NAMED( "StarGazerSerialIO", "Serial port data read but no callback specified!\n" );
			}

			// Remove the consumed message
			m_readData.erase( m_readData.begin( ), msgEnd );

			msgStart = std::find( m_readData.begin( ), m_readData.end( ), STARGAZER_STX );
			msgEnd = std::find( msgStart, m_readData.end( ), STARGAZER_RTX );
		}
	}
	else
	{
		ROS_ERROR_NAMED( "SerialIO", "Read Error: %s\n", error.message( ).c_str( ) );

		if( error == boost::asio::error::eof )
		{
			Close( );
		}
	}

	if( IsOpen( ) )
	{
		StartAsyncRead( );
	}
}

} /* namespace srs */

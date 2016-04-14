/*
 * SerialIO.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include "SerialIO.h"
#include "Helper.h"
#include <thread>
#include <ros/ros.h>

namespace srs {

SerialIO::SerialIO( bool bGenerateCRC, bool bIncludeLength, char cTerminating, char cEscape,
	std::set<char> setCharsToEscape ) :
	m_bGenerateCRC( bGenerateCRC ),
	m_bIncludeLength( bIncludeLength ),
	m_cTerminating( cTerminating ),
	m_cEscape( cEscape ),
	m_setCharsToEscape( setCharsToEscape ),
	m_Thread( ),

	m_oWork( m_IOService ),
	m_SerialPort( m_IOService ),
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

SerialIO::~SerialIO( )
{
	Close( );

	// Stop the service (and the thread)
	m_IOService.stop( );

	// Clean up the thread
	m_Thread->join( );
}

void SerialIO::Open( const char* pszName, std::function<void(std::vector<char>)> readCallback )
{
	m_readCallback = readCallback;

	m_SerialPort.open( pszName );

	// Setup serial port for 8/N/1 operation @ 115.2kHz
	m_SerialPort.set_option( boost::asio::serial_port::baud_rate( 115200 ) );
	m_SerialPort.set_option( boost::asio::serial_port::flow_control( boost::asio::serial_port::flow_control::none ) );
	m_SerialPort.set_option( boost::asio::serial_port::character_size( 8 ) );
	m_SerialPort.set_option( boost::asio::serial_port::parity( boost::asio::serial_port::parity::none ) );
	m_SerialPort.set_option( boost::asio::serial_port::stop_bits( boost::asio::serial_port::stop_bits::one ) );

	StartAsyncRead( );
}

bool SerialIO::IsOpen( ) const
{
	return m_SerialPort.is_open( );
}

void SerialIO::Close( )
{
	m_SerialPort.close( );
}

void SerialIO::Write( const std::vector<char>& buffer )
{
	if( IsOpen( ) )
	{
		// Do all data operations in serial processing thread
		m_IOService.post( std::bind( &SerialIO::WriteInSerialThread, this, buffer, false ) );
	}
	else
	{
		// Throw error
		throw std::runtime_error( "port not open" );
	}
}

void SerialIO::WriteRaw( const std::vector<char>& buffer )
{
	if( IsOpen( ) )
	{
		// Do all data operations in serial processing thread
		m_IOService.post( std::bind( &SerialIO::WriteInSerialThread, this, buffer, true ) );
	}
	else
	{
		// Throw error
		throw std::runtime_error( "port not open" );
	}
}

void SerialIO::WriteInSerialThread( std::vector<char> buffer, bool bIsRaw )
{
	uint32_t dwStartIndex = m_writeData.size( );

	if( !bIsRaw )
	{
		std::vector<char> payload;

		// Add length
		if( m_bIncludeLength )
		{
			m_writeData.push_back( (uint8_t)buffer.size( ) );
		}
	}

	// Add payload data
	m_writeData.insert( m_writeData.end( ), buffer.begin( ), buffer.end( ) );

	if( !bIsRaw )
	{
		uint8_t cCRC = 0;

		// Escape characters and generate CRC
		for( std::size_t dwIndex = dwStartIndex; dwIndex != m_writeData.size( ); dwIndex++ )
		{
			cCRC += m_writeData[dwIndex];

			// Escape the character
			if( m_setCharsToEscape.find( m_writeData[dwIndex] ) != m_setCharsToEscape.end( ) )
			{
				m_writeData.insert( m_writeData.begin( ) + dwIndex, m_cEscape );

				// Skip the escaped character
				dwIndex++;
			}
		};

		// Generate CRC
		if( m_bGenerateCRC )
		{
			m_writeData.push_back( -cCRC );
		}

		// Add terminating character
		if( m_cTerminating )
		{
			m_writeData.push_back( m_cTerminating );
		}
	}

	// If we are already trying to write, then wait until we recieve the callback
	if( !m_bIsWriting )
	{
		m_bIsWriting = true;

		m_SerialPort.async_write_some( boost::asio::buffer( m_writeData.data( ), m_writeData.size( ) ),
		    std::bind( &SerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
	}
}

void SerialIO::StartAsyncRead( )
{
	m_SerialPort.async_read_some( boost::asio::buffer( m_ReadBuffer, m_ReadBuffer.size( ) ),
    	std::bind( &SerialIO::OnReadComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
}

void SerialIO::OnWriteComplete( const boost::system::error_code& error, std::size_t size )
{
	std::vector<char>( m_writeData.begin( ) + size, m_writeData.end( ) ).swap( m_writeData );

	ROS_DEBUG_STREAM_NAMED( "SerialIO", "Write: " << ToHex( m_writeData ) );

	if( m_writeData.begin( ) != m_writeData.end( ) )
	{
		m_SerialPort.async_write_some( boost::asio::buffer( m_writeData.data( ), m_writeData.size( ) ),
			std::bind( &SerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
	}
	else
	{
		m_bIsWriting = false;
	}
}

void SerialIO::OnReadComplete( const boost::system::error_code& error, std::size_t size )
{
	if( !error )
	{
        // Combine buffers
		m_readData.insert( m_readData.end( ), m_ReadBuffer.begin( ), m_ReadBuffer.begin( ) + size );

		if( std::find( m_readData.begin( ), m_readData.end( ), m_cTerminating ) == m_readData.end( ) )
		{
			// Don't bother parsing
			return;
		}

        std::vector<char> messageData;

        bool bIsEscaped = false;

        for( char c : m_readData )
        {
			if( c == m_cEscape )
			{
				if( bIsEscaped )
				{
					messageData.push_back( c );

					bIsEscaped = false;
				}
				else
				{
					bIsEscaped = true;
				}
			}
			else if( bIsEscaped || c != m_cTerminating )
			{
				messageData.push_back( c );

				bIsEscaped = false;
			}
			else
			{
				if( m_readCallback )
				{
					ROS_DEBUG_STREAM_NAMED( "SerialIO", "Read Message: " << ToHex( messageData ) );

					m_readCallback( messageData );

					messageData.clear( );
				}
				else
				{
					printf( "Serial port data read but no callback specified!\n" );
				}
			}
        }

		// Remainder of message
		m_readData = messageData;

		char* pszData = m_readData.data( );

		int i = 0;
	}

	StartAsyncRead( );
}

} /* namespace srs */

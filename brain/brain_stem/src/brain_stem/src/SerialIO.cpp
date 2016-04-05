/*
 * SerialIO.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include "SerialIO.h"
#include <thread>


namespace srs {

SerialIO::SerialIO( ) :
	m_IOService( ),
	m_oWork( m_IOService ),
	m_SerialPort( m_IOService ),
	m_bIsWriting( false ),
	m_ReadBuffer( 1024 ),
	m_writeData( ),
	m_readData( )
{
	// Spin up the thread
	m_Thread.reset( new std::thread( [&](){ m_IOService.run( ); } ) );
}

SerialIO::~SerialIO( )
{
	// Stop the service (and the thread)
	m_IOService.stop( );

	m_Thread->join( );
}

void SerialIO::Open( const char* pszName, std::function<void(std::vector<char>)> readCallback )
{
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

void SerialIO::Write( std::vector<char> buffer )
{
	if( IsOpen( ) )
	{
		// Do all data operations in serial processing thread
		m_IOService.post( std::bind( &SerialIO::WriteInSerialThread, this, buffer ) );
	}
	else
	{
		// Throw error
	}
}

void SerialIO::WriteInSerialThread( std::vector<char> buffer )
{
	m_writeData.insert( m_writeData.end( ), buffer.begin( ), buffer.end( ) );

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

        std::vector<char> messageData;

        bool bIsEscaped = false;

        for( char c : m_readData )
        {
			if( c == '\\' )
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
			else if( bIsEscaped || c != '\n' )
			{
				messageData.push_back( c );

				bIsEscaped = false;
			}
			else
			{
				m_readCallback( messageData );

				messageData.clear( );
			}
        }

		// Remainder of message
		m_readData = messageData;
	}

	StartAsyncRead( );
}

} /* namespace srs */

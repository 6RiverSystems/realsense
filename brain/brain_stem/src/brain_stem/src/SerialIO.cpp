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
	m_SerialPort( m_IOService ),
	m_ReadBuffer( ),
	m_WriteBuffer( )
{
	// Spin up the thread
	std::thread run_thread( [&]{ m_IOService.run(); } );
}

SerialIO::~SerialIO( )
{
	// Stop the service (and the thread)
	m_IOService.stop( );
}

void SerialIO::Open( const std::string& strName, std::function<void(std::vector<char>)> readCallback )
{
	m_SerialPort.open( strName );

//    self.usbCmd.port = '/dev/malg'

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
	    boost::asio::write( m_SerialPort, boost::asio::buffer( buffer ) );
    }
}

void SerialIO::StartAsyncRead( )
{
    boost::asio::async_read( m_SerialPort, m_ReadBuffer,
    	std::bind( &SerialIO::OnReadComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
}

void SerialIO::OnReadComplete( const boost::system::error_code& error, std::size_t size )
{
	if( !error )
	{
        std::vector<char> data( size );

        std::istream is( &m_ReadBuffer );
        is.read( &(data[0]), size );

        // Combine buffers
        m_readData.insert( m_readData.end( ), data.begin( ), data.end( ) );

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
				m_readCallback( data );
				messageData.erase( messageData.begin( ), messageData.end( ) );
			}

			// Remainder of message
			m_readData = messageData;
        }

		StartAsyncRead( );
	}
}

} /* namespace srs */

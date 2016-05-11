/*
 * SerialIO.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/utils/Logging.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <thread>
#include <ros/ros.h>

namespace srs
{

SerialIO::SerialIO( ) :
	m_Thread( ),
	m_serialThreadId( ),
	m_oWork( m_IOService ),
	m_SerialPort( m_IOService ),
	m_strSerialPort( ),
	m_bIsSerialOpen( false ),
	m_fRetryTimeout( 1.0 ),
	m_bIsWriting( false ),
	m_readBuffer( 1024 ),
	m_writeBuffer( ),
	m_readState( READ_STATE::DEFAULT ),
	m_cCRC( 0 ),
	m_readPartialData( ),
	m_readCallback( ),
	m_bEnableCRC( false ),
	m_bHasLeading( false ),
	m_cLeading( 0 ),
	m_bHasTerminating( false ),
	m_cTerminating( 0 ),
	m_bHasEscape( false ),
	m_cEscape( 0 ),
	m_firstByteDelay( ),
	m_interByteDelay( )
#if defined( ENABLE_TEST_FIXTURE )
	,
	m_highrezclk( ),
	m_lastTime( std::chrono::milliseconds( 0 ) ),
	m_messageTiming( ),
	m_vecMessageTiming( )
#endif
{
	m_oTimer.reset( new boost::asio::deadline_timer( m_IOService, boost::posix_time::seconds( 5 ) ) );

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

void SerialIO::Open( const char* pszName, ReadCallbackFn readCallback )
{
	if( !IsOpen( ) )
	{
		std::condition_variable condition;
		std::mutex mutex;

		m_IOService.post( [&]( )
			{
				m_strSerialPort = pszName;

				m_readCallback = readCallback;

				m_serialThreadId = std::this_thread::get_id( );

				OnCheckSerialPort( false );

				condition.notify_one( );
			} );


		std::unique_lock<std::mutex> lock( mutex );

		condition.wait( lock );
	}
	else
	{
		ROS_ERROR_STREAM_NAMED( "SerialIO", "Attempt to open serial port but it is already open." );
	}
}

bool SerialIO::IsOpen( ) const
{
	return m_SerialPort.is_open( );
}

void SerialIO::Close( )
{
	m_SerialPort.close( );

	m_strSerialPort = "";
}

////////////////////////////////////////////////////////////////////////////////////////////
// Serial IO Configuration
////////////////////////////////////////////////////////////////////////////////////////////

void SerialIO::SetRetryTimeout( float fRetryTimeout )
{
	m_fRetryTimeout = fRetryTimeout;
}

void SerialIO::EnableCRC( bool bGenerateCRC )
{
	m_bEnableCRC = bGenerateCRC;
}

void SerialIO::SetLeadingCharacter( char cLeading )
{
	m_cLeading = cLeading;

	m_bHasLeading = true;
}

void SerialIO::SetTerminatingCharacter( char cTerminating )
{
	m_cTerminating = cTerminating;

	m_bHasTerminating = true;
}

void SerialIO::SetEscapeCharacter( char cEscape )
{
	m_cEscape = cEscape;

	m_bHasEscape = true;
}

void SerialIO::SetFirstByteDelay( std::chrono::microseconds firstByteDelay )
{
	m_firstByteDelay = firstByteDelay;
}

std::chrono::microseconds SerialIO::GetFirstByteDelay( )
{
	return m_firstByteDelay;
}

void SerialIO::SetByteDelay( std::chrono::microseconds byteDelay )
{
	m_byteDelay = byteDelay;
}

std::chrono::microseconds SerialIO::GetByteDelay( )
{
	return m_byteDelay;
}

void SerialIO::Write( const std::vector<char>& buffer )
{
	if( IsOpen( ) )
	{
		// Do all data operations in serial processing thread
		m_IOService.post( std::bind( &SerialIO::WriteInSerialThread, this, buffer ) );
	}
	else
	{
		// Throw error
		throw std::runtime_error( "port not open" );
	}
}

void SerialIO::WriteInSerialThread( std::vector<char> writeBuffer )
{
	assert( m_serialThreadId == std::this_thread::get_id( ) );
//
//	ROS_DEBUG_STREAM_NAMED( "SerialIO", "Write Complete: " <<
//		std::string( writeBuffer.begin( ), writeBuffer.end( ) ) );

	// Add leading character
	if( m_bHasLeading )
	{
		m_writeBuffer.push_back( m_cLeading );
	}

	uint8_t cCRC = 0;

	// Function to add an escaped char (if necessary) to the array
	auto addEscapedCharacter =
		[&](const char& cChar )
		{
			if( m_bHasEscape )
			{
				if( ( cChar == m_cEscape )					 ||
					( m_bHasLeading && cChar == m_cLeading ) ||
					( m_bHasTerminating && cChar == m_cTerminating ) )
				{
					m_writeBuffer.push_back( m_cEscape );
				}
			}

			m_writeBuffer.push_back( cChar );
		};

	for( const char& cChar : writeBuffer )
	{
		cCRC += cChar;

		addEscapedCharacter( cChar );
	}

	// Add CRC
	if( m_bEnableCRC )
	{
		addEscapedCharacter( -cCRC );
	}

	// Add terminating character
	if( m_bHasTerminating )
	{
		m_writeBuffer.push_back( m_cTerminating );
	}

	// If we are already trying to write, then wait until we receive the callback
	if( !m_bIsWriting )
	{
		m_bIsWriting = true;

		m_interByteDelay = m_firstByteDelay;

		// If we have a first byte delay only write one byte
		size_t writeSize = m_interByteDelay.count( ) ?  1 : m_writeBuffer.size( );

//		ROS_DEBUG_STREAM_NAMED( "SerialIO", "Write Bytes: " << writeSize );

		m_SerialPort.async_write_some( boost::asio::buffer( m_writeBuffer.data( ), writeSize ),
			std::bind( &SerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
	}
}

void SerialIO::StartAsyncRead( )
{
	m_SerialPort.async_read_some( boost::asio::buffer( m_readBuffer, m_readBuffer.size( ) ),
		std::bind( &SerialIO::OnReadComplete, this, std::placeholders::_1, std::placeholders::_2 ) );
}

void SerialIO::OnWriteComplete( const boost::system::error_code& error, std::size_t size )
{
	assert( m_serialThreadId == std::this_thread::get_id( ) );
//
//	ROS_DEBUG_STREAM_NAMED( "SerialIO", "Write Complete: " <<
//		std::string( m_writeBuffer.begin( ), m_writeBuffer.begin( ) + size ) );

	std::vector<char>( m_writeBuffer.begin( ) + size, m_writeBuffer.end( ) ).swap( m_writeBuffer );

	if( m_writeBuffer.begin( ) != m_writeBuffer.end( ) )
	{
//		ROS_DEBUG_STREAM_NAMED( "SerialIO", "Write Bytes: sleep for " << m_interByteDelay.count( ) );

		std::this_thread::sleep_for( m_interByteDelay );

		m_interByteDelay = m_byteDelay;

		size_t writeSize = m_interByteDelay.count( ) ? 1 : m_writeBuffer.size( );

		m_SerialPort.async_write_some( boost::asio::buffer( m_writeBuffer.data( ), writeSize ),
			std::bind( &SerialIO::OnWriteComplete, this, std::placeholders::_1, std::placeholders::_2 ) );

		// Reset the first byte delay if we encounter a start or end of message
		if( size == 1 && m_firstByteDelay.count( ) )
		{
			if( m_bHasLeading )
			{
				if( m_cLeading == m_writeBuffer[0] )
				{
					m_interByteDelay = m_firstByteDelay;
				}
			}
			else if( m_bHasTerminating )
			{
				if( m_cTerminating == m_writeBuffer[0] )
				{
					m_interByteDelay = m_firstByteDelay;
				}
			}
		}
	}
	else
	{
		m_bIsWriting = false;
	}
}

void SerialIO::OnReadComplete( const boost::system::error_code& error, std::size_t size )
{
	#if defined( ENABLE_TEST_FIXTURE )

		auto now = m_highrezclk.now( );

	#endif

	assert( m_serialThreadId == std::this_thread::get_id( ) );

	if( !error )
	{
		// Start with the partially parsed message
		std::vector<char> messageData( m_readPartialData.begin( ), m_readPartialData.end( ) );

//		ROS_DEBUG_STREAM_NAMED( "SerialIO", "Left over read data: " << ToHex( messageData ) );

		auto changeState =
			[&]( const READ_STATE& eNewState )
			{
				#if defined( ENABLE_TEST_FIXTURE )
					if( m_readState == READ_STATE::DEFAULT &&
						eNewState == READ_STATE::IN_MESSAGE )
					{
//						ROS_DEBUG_NAMED( "SerialIO", "Time reset" );

						m_lastTime = now;
					}
				#endif

				m_readState = eNewState;
			};

		// Function to add an escaped char (if necessary) to the array
		auto addCharacter =
			[&]( const char& cChar )
			{
				messageData.push_back( cChar );

				m_cCRC += cChar;
			};

		for( auto iter = m_readBuffer.begin( ); iter < m_readBuffer.begin( ) + size; iter++ )
		{
			#if defined( ENABLE_TEST_FIXTURE )

//				ROS_DEBUG_NAMED( "SerialIO", "Time between reads %d: %d ", messageData.size( ), std::chrono::duration_cast<std::chrono::microseconds>(now - m_lastTime).count( ) );

				m_messageTiming.push_back( std::chrono::duration_cast<std::chrono::microseconds>(now - m_lastTime) );

				m_lastTime = now;

			#endif

			// If we do not have a leading character, then we are always in a message
			if( m_readState == READ_STATE::DEFAULT && !m_bHasLeading )
			{
				changeState( READ_STATE::IN_MESSAGE );
			}

//			ROS_ERROR_NAMED( "SerialIO", "Char: %.2x", *iter );

			if( m_bHasEscape &&
				*iter == m_cEscape )
			{
				if( m_readState == READ_STATE::IN_MESSAGE_ESCAPED )
				{
					addCharacter( *iter );

					changeState( READ_STATE::IN_MESSAGE );
				}
				else
				{
					changeState( READ_STATE::IN_MESSAGE_ESCAPED );
				}
			}
			else if( m_readState == READ_STATE::DEFAULT )
			{
				// Should never get into this case if we are not a leading character
				assert( m_bHasLeading );

				// Only start a message when we encounter a leading character
				if( *iter == m_cLeading )
				{
					changeState( READ_STATE::IN_MESSAGE );
				}
			}
			else if( m_readState == READ_STATE::IN_MESSAGE_ESCAPED ||
				*iter != m_cTerminating )
			{
				addCharacter( *iter );

				changeState( READ_STATE::IN_MESSAGE );
			}
			else
			{
				if( messageData.size( ) > 0 )
				{
					if( messageData[0] == '<' )
					{
						m_cCRC = 0;
					}
					else if( m_bEnableCRC )
					{
						messageData.pop_back( );
					}
					else
					{
						// We are not using a CRC
						m_cCRC = 0;
					}

					if( m_cCRC == 0 )
					{
						if( m_readCallback )
						{
//							ROS_ERROR_STREAM_NAMED( "SerialIO", "ReadData: " <<
//								std::string( messageData.begin( ), messageData.end( ) ) );

							m_readCallback( messageData );
						}
						else
						{
							ROS_ERROR_NAMED( "SerialIO", "Serial port data read but no callback specified!\n" );
						}
					}
					else
					{
						ROS_ERROR_STREAM_NAMED( "SerialIO", "Invalid CRC (" << m_cCRC << "): " << ToHex( messageData ) );
					}
				}
				else
				{
					ROS_ERROR_STREAM_NAMED( "SerialIO", "Empty Message" );
				}

				messageData.clear( );

				m_cCRC = 0;

				changeState( READ_STATE::DEFAULT );

				#if defined( ENABLE_TEST_FIXTURE )

					m_vecMessageTiming.push_back( m_messageTiming );

					m_messageTiming.clear( );

				#endif
			}
		}

//		ROS_DEBUG_STREAM_NAMED( "SerialIO", "Left over read data: " << ToHex( m_readPartialData ) );

		// Left over partial message
		m_readPartialData = messageData;
	}
	else
	{
		if( IsOpen( ) )
		{
			ROS_ERROR_NAMED( "SerialIO", "Read Error: %s", error.message( ).c_str( ) );

			if( error == boost::asio::error::eof )
			{
				Close( );
			}
		}
	}

	if( IsOpen( ) )
	{
		StartAsyncRead( );
	}
}

bool SerialIO::OnCheckSerialPort( bool bInitialCheck, const boost::system::error_code& e )
{
	assert( m_serialThreadId == std::this_thread::get_id( ) );

	// Save current state (so we don't spam debug log on reconnect)
	bool bIsSerialOpen = m_bIsSerialOpen;

	m_bIsSerialOpen = IsOpen( );

	std::string strError;

	if( !m_bIsSerialOpen )
	{
		try
		{
			// Anonymous function call the message processor in the main ros thread
			m_SerialPort.open( m_strSerialPort );

			// Setup serial port for 8/N/1 operation @ 115.2kHz
			m_SerialPort.set_option( boost::asio::serial_port::baud_rate( 115200 ) );
			m_SerialPort.set_option( boost::asio::serial_port::flow_control( boost::asio::serial_port::flow_control::none ) );
			m_SerialPort.set_option( boost::asio::serial_port::character_size( 8 ) );
			m_SerialPort.set_option( boost::asio::serial_port::parity( boost::asio::serial_port::parity::none ) );
			m_SerialPort.set_option( boost::asio::serial_port::stop_bits( boost::asio::serial_port::stop_bits::one ) );

			m_readState = READ_STATE::DEFAULT;

			m_cCRC = 0;

			#if defined( ENABLE_TEST_FIXTURE )

				m_vecMessageTiming.clear( );

			#endif

			m_bIsSerialOpen = true;
		}
		catch( const std::exception& e )
		{
			strError = e.what( );
		}
		catch( ... )
		{
			strError = "Unknown error";
		}

		if( !m_bIsSerialOpen && bInitialCheck )
		{
			ROS_DEBUG( "Error connecting to serial port (%s): %s (Retry: %.2f)", m_strSerialPort.c_str( ), strError.c_str( ),
				m_fRetryTimeout );
		}
	}

	if( bIsSerialOpen != m_bIsSerialOpen )
	{
		if( m_bIsSerialOpen == true )
		{
			ROS_INFO( "Connected to serial port" );
		}
		else
		{
			ROS_ERROR( "Disconnected from serial port: %s (Retry: %.2f)", strError.c_str( ), m_fRetryTimeout );
		}
	}

	// If we failed to open the serial port keep trying
	if( m_bIsSerialOpen )
	{
		StartAsyncRead( );
	}
	else
	{
		m_oTimer->async_wait( [&]( const boost::system::error_code& e )
			{
				OnCheckSerialPort( false, e );
			} );
	}

	return m_bIsSerialOpen;
}

} /* namespace srs */

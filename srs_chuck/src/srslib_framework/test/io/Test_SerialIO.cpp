/*
 * SerialIO_test.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include <srslib_framework/io/SerialIO.hpp>
#include "gtest/gtest.h"
#include <cstdlib>
#include <unistd.h>
#include <queue>
#include <condition_variable>
#include <boost/range/irange.hpp>
#include <ros/ros.h>
#include <boost/timer.hpp>

using namespace srs;

namespace {

const std::string g_strPort1 = "/tmp/pty1";
const std::string g_strPort2 = "/tmp/pty2";

typedef std::vector<char> IOBuffer;

std::vector<IOBuffer> g_vecTestData =
{
	{ 'H', 'e', 'l', 'l', 'o' },
	{ '6', 'R', 'i', 'v', 'e', 'r', 's' },
	{ '\\', 'A', 'A', '\n', 'B', 'B', '"' },
	{ 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F },
	{ 'A', 'B', 'C', 'D', 'E', 'F', 'G', '\n' },
	{ 0x01, 0x02, 0x03, 0x04 },  // Special case where CRC == '\n'
	{ 0x10, 0x10, 0x10, 0x10, 0x10, 0x0C }  // Special case where CRC == '\\'
};

class SocatRunner
{
private:
	bool	m_bIsRunning;

public:

	SocatRunner( ) :
		m_bIsRunning( false )
	{
		Stop( );

		RemovePseudoTerminals( );

		usleep( 1000000 );
	}

	~SocatRunner( )
	{
		Stop( );

		RemovePseudoTerminals( );
	}

	void Start( )
	{
		if( !m_bIsRunning )
		{
			// Open up two linked pseudo serial ports
			std::string strStartCommand;
			strStartCommand += "touch " + g_strPort1 + " &&";
			strStartCommand += "touch " + g_strPort2 + " &&";
			strStartCommand += "socat -d -d pty,link=" + g_strPort1 + ",raw,echo=0 pty,link=" + g_strPort2 + ",raw,echo=0";
			strStartCommand += " &"; // Don't wait (background)

			// Startup socat
			printf ( "Starting socat: %s\n", strStartCommand.c_str( ) );
			std::system( strStartCommand.c_str( ) );

			m_bIsRunning = true;
		}
	}

	void Stop( )
	{
		if( m_bIsRunning )
		{
			// Kill socat
			std::string strStopCommand( "killall -1 socat" );

			printf( "Stopping socat: \"%s\"\n", strStopCommand.c_str( ) );
			std::system( strStopCommand.c_str( ) );

			m_bIsRunning = false;
		}
	}

	void RemovePseudoTerminals( )
	{
		std::string strRemoveTempTerminal( "rm " );
		strRemoveTempTerminal += g_strPort1;
		strRemoveTempTerminal += " ";
		strRemoveTempTerminal += g_strPort2;

		std::system( strRemoveTempTerminal.c_str( ) );
	}

} g_socat;

class SerialIOTest : public ::testing::Test
{
public:

	enum class CONFIG
	{
		RAW,
		BRAIN_STEM,
		STAR_GAZER
	};

	static const char STARGAZER_LEADING = '~';
	static const char STARGAZER_TERMINATING = '`';

public:

	SerialIO						m_serial1;

	SerialIO						m_serial2;

	std::queue<std::vector<char>>	m_readData1;

	std::queue<std::vector<char>>	m_readData2;

	std::mutex              		m_mutex1;

	std::mutex              		m_mutex2;

	std::condition_variable			m_condition1;

	std::condition_variable			m_condition2;

	std::vector<CONFIG>				m_testConfigs;

public:
	SerialIOTest( ) :
		m_serial1( "test1", g_strPort1.c_str() ),
		m_serial2( "test2", g_strPort2.c_str() )
	{
		m_testConfigs.push_back( CONFIG::BRAIN_STEM );
		m_testConfigs.push_back( CONFIG::STAR_GAZER );
	}

	void OpenSerialPort( SerialIO& serial, CONFIG eConfig,
		void (SerialIOTest::* callback) (std::vector<char>) )
	{;
		auto connectionCallback = [](bool bIsConnected) { };

		auto readCallback = std::bind( callback, this, std::placeholders::_1 );

		serial.open( connectionCallback, readCallback );

		switch( eConfig )
		{
			case CONFIG::RAW:
			{
				// Nothing
			}
			break;

			case CONFIG::BRAIN_STEM:
			{
				serial.enableCRC( true );
				serial.setTerminatingCharacter( '\n' );
				serial.setEscapeCharacter( '\\' );
				serial.setFirstByteDelay( std::chrono::microseconds( ) );
				serial.setByteDelay( std::chrono::microseconds( ) );
			}
			break;

			case CONFIG::STAR_GAZER:
			{
				serial.setLeadingCharacter( STARGAZER_LEADING );
				serial.setTerminatingCharacter( STARGAZER_TERMINATING );
				serial.setFirstByteDelay( std::chrono::microseconds( 30000 ) );
				serial.setByteDelay( std::chrono::microseconds( 5000 ) );
			}
			break;

		}

		EXPECT_TRUE( serial.isOpen( ) );
	}

	void OpenSerialPort1( CONFIG eConfig = CONFIG::BRAIN_STEM )
	{
		OpenSerialPort( m_serial1, eConfig, &SerialIOTest::ReadMessageFrom2 );
	}

	void OpenSerialPort2( CONFIG eConfig = CONFIG::BRAIN_STEM )
	{
		OpenSerialPort( m_serial2, eConfig, &SerialIOTest::ReadMessageFrom1 );
	}

	void OpenSerialPorts( CONFIG eConfig = CONFIG::BRAIN_STEM )
	{
		OpenSerialPort1( eConfig );

		OpenSerialPort2( eConfig );
	}

	void CloseSerialPort( SerialIO& serial )
	{
		serial.close( );

		EXPECT_TRUE( !serial.isOpen( ) );
	}

	void CloseSerialPorts( )
	{
		CloseSerialPort( m_serial1 );
		CloseSerialPort( m_serial2 );
	}

	void SetUp( )
	{

	}

	void TearDown( )
	{
		std::queue<std::vector<char>> emptyQueue;

		m_readData1.swap( emptyQueue );;

		m_readData2.swap( emptyQueue );

		m_serial1.close( );

		m_serial2.close( );
	}

	void ReadMessageFrom1( std::vector<char> buffer )
	{
		m_readData2.push( buffer );

		std::unique_lock<std::mutex> lock( m_mutex2 );

		m_condition2.notify_one( );
	}

	void ReadMessageFrom2( std::vector<char> buffer )
	{
		m_readData1.push( buffer );

		std::unique_lock<std::mutex> lock( m_mutex1 );

		m_condition1.notify_one( );
	}

	void WaitForData1( size_t numberOfMessages )
	{
		WaitForData( numberOfMessages, m_mutex1, m_condition1, m_readData1 );
	}

	void WaitForData2( size_t numberOfMessages )
	{
		WaitForData( numberOfMessages, m_mutex2, m_condition2, m_readData2 );
	}

	void WaitForData( size_t numberOfMessages, std::mutex& mutex, std::condition_variable& condition,
		std::queue<std::vector<char>>& queueData )
	{
		// Wait for for messages to come in or timeout
		while( true )
		{
			std::unique_lock<std::mutex> lock( mutex );

			if( condition.wait_until( lock, std::chrono::steady_clock::now( ) + std::chrono::milliseconds( 1000 ) ) == std::cv_status::timeout )
			{
				break;
			}
			else
			{
				if( queueData.size( ) == numberOfMessages )
				{
					break;
				}
			}
		}
	}

	void CheckTiming( SerialIO& serial )
	{
		std::chrono::microseconds firstByteDelay = serial.getFirstByteDelay( );

		std::chrono::microseconds byteDelay = serial.getByteDelay( );

		if( firstByteDelay == std::chrono::microseconds( ) )
		{
			firstByteDelay = byteDelay;
		}

		std::vector<SerialIO::MessageTiming> vecMessageTiming = serial.getTimingInfo( );

		for( auto messageTiming : vecMessageTiming )
		{
			for( int i = 0; i < messageTiming.size( ); i++ )
			{
				// Make sure time constraints are met
				if( i == 1 )
				{
					ASSERT_GE( messageTiming[i].count( )*1.1, firstByteDelay.count( ) );
				}
				else if( i > 1)
				{
					ASSERT_GE( messageTiming[i].count( )*1.1, byteDelay.count( ) );
				}
			}
		}
	}

	void CheckTiming( )
	{
		CheckTiming( m_serial1 );
		CheckTiming( m_serial2 );
	}

	~SerialIOTest( )
	{

	}

   // put in any custom data members that you need
};

TEST_F( SerialIOTest, TestSpinUntilOpen )
{
	// Try once when it is closed, then spin until it connects (wait for 3 seconds)
	for( int i : boost::irange( 0, 300 ) )
	{
		m_serial1.open( [](bool bIsConnected) { },
			std::bind( &SerialIOTest::ReadMessageFrom2, this, std::placeholders::_1 ) );

		// If we are able to open when socat is not running, we have failed
		if( i == 0 )
		{
	    	ASSERT_FALSE( m_serial1.isOpen( ) );
		}

		// Start it after the first failure
		if( i == 0 )
		{
			g_socat.Start( );
		}
		else
		{
			if( m_serial1.isOpen( ) )
			{
				break;
			}
		}

		// Wait for 10ms
		usleep( 100000 );
	}

	EXPECT_TRUE( m_serial1.isOpen( ) );
}

TEST_F( SerialIOTest, TestOpen )
{
	OpenSerialPorts( );
}

TEST_F( SerialIOTest, TestClose )
{
	OpenSerialPorts( );

	CloseSerialPorts( );
}

TEST_F( SerialIOTest, TestBasicReadWrite )
{
	for( auto iter : m_testConfigs )
	{
		OpenSerialPorts( iter );

		for( auto data : g_vecTestData )
		{
			m_serial1.write( data );
		}

		WaitForData2( g_vecTestData.size( ) );

		CloseSerialPorts( );

		CheckTiming( m_serial2 );

		ASSERT_EQ( g_vecTestData.size( ), m_readData2.size( ) );

		for( auto data : g_vecTestData )
		{
			EXPECT_EQ( m_readData2.front( ), data );

			m_readData2.pop( );
		}

		CloseSerialPorts( );
	}
}

TEST_F( SerialIOTest, TestDuplexReadWrite )
{
	OpenSerialPorts( );

	for( auto data : g_vecTestData )
	{
		m_serial1.write( data );
		m_serial2.write( data );
	}

	WaitForData1( g_vecTestData.size( ) );
	WaitForData2( g_vecTestData.size( ) );

	ASSERT_EQ( g_vecTestData.size( ), m_readData2.size( ) );

	CloseSerialPorts( );

	CheckTiming( );

	for( auto data : g_vecTestData )
	{
		EXPECT_EQ( m_readData1.front( ), data );
		EXPECT_EQ( m_readData2.front( ), data );

		m_readData1.pop( );
		m_readData2.pop( );
	}
}

TEST_F( SerialIOTest, TestCarryOver )
{
	OpenSerialPort1( CONFIG::RAW );

	OpenSerialPort2( CONFIG::BRAIN_STEM );

	std::vector<char> data( { 'H', 'e', 'l', 'l', 'o' } );

	uint32_t index = 3;

	m_serial1.write( std::vector<char>( data.begin( ), data.begin( ) + index ) );
	m_serial1.write( std::vector<char>( data.begin( ) + index, data.end( ) ) );
	m_serial1.write( std::vector<char>( { (char)-0xF4, '\n' } ) );

	WaitForData2( g_vecTestData.size( ) );

	ASSERT_EQ( m_readData2.size( ), 1 );

	EXPECT_EQ( data, m_readData2.front( ) );
}

TEST_F( SerialIOTest, TestInvalidCRC )
{
	OpenSerialPort1( CONFIG::RAW );

	OpenSerialPort2( CONFIG::BRAIN_STEM );

	std::vector<char> data( { 'H', 'e', 'l', 'l', 'o' } );

	uint32_t index = 3;

	m_serial1.write( std::vector<char>( data.begin( ), data.begin( ) + index ) );
	m_serial1.write( std::vector<char>( data.begin( ) + index, data.end( ) ) );
	m_serial1.write( std::vector<char>( { 'x', '\n' } ) );

	WaitForData2( g_vecTestData.size( ) );

	ASSERT_NE( m_readData2.size( ), 1 );

	EXPECT_NE( data, m_readData2.front( ) );
}

TEST_F( SerialIOTest, TestStartInMiddleOfMessage )
{
	OpenSerialPort1( CONFIG::RAW );

	OpenSerialPort2( CONFIG::STAR_GAZER );

	for( auto data : g_vecTestData )
	{
		// Garbage data to ignore
		m_serial1.write( data );

		m_serial1.write( std::vector<char>( { STARGAZER_LEADING } ) );

		m_serial1.write( data );

		m_serial1.write( std::vector<char>( { STARGAZER_TERMINATING } ) );
	}

	WaitForData2( g_vecTestData.size( ) );

	for( auto data : g_vecTestData )
	{
		EXPECT_EQ( m_readData2.front( ), data );

		m_readData2.pop( );
	}
}

TEST_F( SerialIOTest, TestCloseWhileReadWrite )
{
	OpenSerialPorts( );

	std::vector<char> data;

	std::string str64Bytes( "1234567890123456789012345678901234567890123456789012345678901234" );

	// 65k of data
	for( int i : boost::irange( 0, 1024 ) )
	{
		data.insert( data.end( ), str64Bytes.begin( ), str64Bytes.end( ) );
	}

	std::vector<char> data1( data.begin( ), data.end( ) );

	m_serial1.write( data );

	m_serial2.write( data );

	// Closing while reading and writing is perfectly fine
	m_serial1.close( );

	m_serial2.close( );
}

TEST_F( SerialIOTest, TestBandwidth )
{
	OpenSerialPorts( );

	std::vector<char> data;

	std::string str64Bytes( "1234567890123456789012345678901234567890123456789012345678901234" );

	// 65k of data
	for( int i : boost::irange( 0, 1024 ) )
	{
		// Add in increments of 64 bytes
		data.insert( data.end( ), str64Bytes.begin( ), str64Bytes.end( ) );
	}

	boost::timer oTimer;

	m_serial1.write( data );

	WaitForData2( 1 );

	double dfTime = oTimer.elapsed( );

	double dfBytes = (double)data.size( );

	double dfBandwidth =  dfBytes / dfTime;

	EXPECT_EQ( m_readData2.front( ), data );

	EXPECT_GT( dfBandwidth, 115200.0f * 0.90 );

	ROS_DEBUG( "SerialIO Bandwidth: %f bytes/sec", dfBandwidth );
}

}  // namespace

int main(int argc, char **argv)
{
	g_socat.Stop( );

	::testing::InitGoogleTest( &argc, argv );

	int success = RUN_ALL_TESTS( );

	g_socat.Stop( );

	return success;
}

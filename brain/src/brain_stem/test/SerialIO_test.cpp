/*
 * SerialIO_test.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include "SerialIO.h"
#include "gtest/gtest.h"
#include <cstdlib>
#include <unistd.h>
#include <queue>
#include <condition_variable>
#include <boost/range/irange.hpp>

using namespace srs;

namespace {

const std::string g_strPort1 = "/tmp/pty1";
const std::string g_strPort2 = "/tmp/pty2";

class SerialIOTest : public ::testing::Test
{
public:

	SerialIO						m_Serial1;

	SerialIO						m_Serial2;

	std::queue<std::vector<char>>	m_readData1;

	std::queue<std::vector<char>>	m_readData2;

	std::mutex              		m_mutex1;

	std::mutex              		m_mutex2;

	std::condition_variable			m_condition1;

	std::condition_variable			m_condition2;

public:
	SerialIOTest( )
	{

	}

	void OpenSerialPort( SerialIO& serial, std::string strPort,
		void (SerialIOTest::* callback) (std::vector<char>) )
	{
	    try
	    {
	    	serial.Open( strPort.c_str( ), std::bind( callback, this,
	    		std::placeholders::_1 ) );

	    	EXPECT_TRUE( serial.IsOpen( ) );
	    }
	    catch( const std::exception& e )
	    {
	        FAIL( ) << "Open failed: " <<  e.what( );
	    }
	    catch( ... )
	    {
	        FAIL( ) << "Open failed: unknown exception";
	    }
	}

	void OpenSerialPort1( )
	{
		OpenSerialPort( m_Serial1, g_strPort1, &SerialIOTest::ReadMessageFrom2 );
	}

	void OpenSerialPort2( )
	{
		OpenSerialPort( m_Serial2, g_strPort2, &SerialIOTest::ReadMessageFrom1 );
	}

	void SetUp( )
	{

	}

	void TearDown( )
	{

	}

	bool CompareBuffers( const std::vector<char>& buffer1, const std::vector<char>& buffer2 )
	{
		return buffer1.size( ) == buffer2.size( ) &&
			memcmp( buffer1.data( ), buffer2.data( ), buffer1.size( ) );
	}

	void ReadMessageFrom1( std::vector<char> buffer )
	{
		m_readData2.push( buffer );

		std::unique_lock<std::mutex> lock( m_mutex1 );

		m_condition1.notify_one( );
	}

	void ReadMessageFrom2( std::vector<char> buffer )
	{
		m_readData1.push( buffer );

		std::unique_lock<std::mutex> lock( m_mutex2 );

		m_condition2.notify_one( );
	}

	~SerialIOTest( )
	{

	}

   // put in any custom data members that you need
};

TEST_F( SerialIOTest, OpenInvalidSerialPort )
{
    try
    {
    	m_Serial1.Open( "/foobar", std::bind( &SerialIOTest::ReadMessageFrom2, this,
    		std::placeholders::_1 ) );

    	FAIL( ) << "Expected std::exception";
    }
    catch( boost::system::system_error const & err )
    {
    	const char* pszError = err.what( );

    	ASSERT_STREQ( "open: No such file or directory", pszError );
    }
    catch( ... )
    {
        FAIL( ) << "Expected std::exception";
    }
}

TEST_F( SerialIOTest, OpenSuccess )
{
	OpenSerialPort1( );

	OpenSerialPort2( );
}

TEST_F( SerialIOTest, Close )
{
	OpenSerialPort1( );

	OpenSerialPort2( );
}

TEST_F( SerialIOTest, TestSimpleReadWrite )
{
	OpenSerialPort1( );

	OpenSerialPort2( );

	std::string string1( "Hello" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_Serial1.Write( data1 );

	std::unique_lock<std::mutex> lock( m_mutex1 );

	m_condition1.wait( lock );

	std::vector<char> expectedData( { 72, 101, 108, 108, 111, -12 } );

	EXPECT_EQ( m_readData2.front( ), expectedData );
}

TEST_F( SerialIOTest, TestMultipleReadWrites )
{
	OpenSerialPort1( );

	OpenSerialPort2( );

	std::string string1( "Hello" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_Serial1.WriteRaw( std::vector<char>( ) );

	m_Serial1.WriteRaw( data1 );
	usleep( 5000 );
	m_Serial1.WriteRaw( data1 );
	usleep( 10000 );
	m_Serial1.WriteRaw( data1 );
	usleep( 15000 );
	m_Serial1.WriteRaw( std::vector<char>( { '\n' } ) );

	std::unique_lock<std::mutex> lock( m_mutex1 );

	m_condition1.wait( lock );

	std::string strExpected( "HelloHelloHello" );
	std::vector<char> expectedData( strExpected.begin( ), strExpected.end( ) );

	EXPECT_EQ( m_readData2.front( ), expectedData );
}


TEST_F( SerialIOTest, TestMultipleMessages )
{
	OpenSerialPort1( );

	OpenSerialPort2( );

	std::string string1( "HelloHelloHelloHelloHelloHelloHelloHelloHello" );
	std::vector<char> data1( string1.begin( ), string1.end( ) );

	m_Serial1.WriteRaw( data1 );
	m_Serial1.WriteRaw( std::vector<char>( { '\n' } ) );

	usleep( 5000 );

	m_Serial1.WriteRaw( data1 );
	m_Serial1.WriteRaw( std::vector<char>( { '\n' } ) );

	usleep( 10000 );

	m_Serial1.WriteRaw( data1 );
	m_Serial1.WriteRaw( std::vector<char>( { '\n' } ) );

	usleep( 15000 );

	m_Serial1.WriteRaw( data1 );
	m_Serial1.WriteRaw( std::vector<char>( { '\n' } ) );

	usleep( 15000 );

	m_Serial1.WriteRaw( data1 );
	m_Serial1.WriteRaw( std::vector<char>( { '\n' } ) );

	for( int i : boost::irange( 0, 5 ) )
	{
		std::unique_lock<std::mutex> lock( m_mutex1 );

		// Wait for next message if necessary
		if( m_readData2.size( ) == 0 )
		{
			m_condition1.wait( lock );
		}

		EXPECT_EQ( m_readData2.front( ), data1 );

		m_readData2.pop( );
	}
}

}  // namespace

int main(int argc, char **argv)
{
	time_t rawtime;
	struct tm * timeinfo;

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	printf ( "\nCurrent local time and date: %s", asctime (timeinfo) );

	// Open up two linked pseudo serial ports
	std::string strCommand;
	strCommand += "touch " + g_strPort1 + " &&";
	strCommand += "touch " + g_strPort2 + " &&";
	strCommand += "socat -d -d pty,link=" + g_strPort1 + ",raw,echo=0 pty,link=" + g_strPort2 + ",raw,echo=0";
	strCommand += " &"; // Don't wait (background)

	std::system( strCommand.c_str( ) );

	// Sleep for 200ms to let socat startup
	usleep( 200000 );

	::testing::InitGoogleTest( &argc, argv );

	int success = RUN_ALL_TESTS( );

	std::system( "killall socat" );

	return success;

}

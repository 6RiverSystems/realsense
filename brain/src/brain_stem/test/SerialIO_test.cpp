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
#include <boost/date_time/posix_time/posix_time.hpp>

using namespace srs;

namespace {

const std::string g_strPort1 = "/tmp/pty1";
const std::string g_strPort2 = "/tmp/pty2";

class SerialIOTest : public ::testing::Test
{
public:

	SerialIO	m_Serial1;

	SerialIO	m_Serial2;

public:
	SerialIOTest( )
	{

	}

	void OpenSerialPort( SerialIO& serial, void (SerialIOTest::* callback) (std::vector<char>) )
	{
	    try
	    {
	    	serial.Open( g_strPort1.c_str( ), std::bind( callback, this,
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
		OpenSerialPort( m_Serial1, &SerialIOTest::ReadCallback1 );
	}

	void OpenSerialPort2( )
	{
		OpenSerialPort( m_Serial2, &SerialIOTest::ReadCallback2 );
	}

	void SetUp( )
	{

	}

	void TearDown( )
	{

	}

	void ReadCallback1( std::vector<char> buffer )
	{
		printf( "ReadCallback1: %s", buffer.data( ) );
	}

	void ReadCallback2( std::vector<char> buffer )
	{
		printf( "ReadCallback2: %s", buffer.data( ) );
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
    	m_Serial1.Open( "/foobar", std::bind( &SerialIOTest::ReadCallback1, this,
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


TEST_F( SerialIOTest, TestReadWrite )
{
	OpenSerialPort1( );

	OpenSerialPort2( );

	std::string data( "Hello" );
	m_Serial1.Write( std::vector<char>( data.begin( ), data.end( ) ) );


	std::string data2( "World" );
	m_Serial2.Write( std::vector<char>( data2.begin( ), data2.end( ) ) );

	usleep( 50000 );
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

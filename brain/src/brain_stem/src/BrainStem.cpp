#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include "SerialIO.h"
#include "MessageProcessor.h"

////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////

using namespace srs;

class GenericCallback : public ros::CallbackInterface
{
private:

	std::function<void()> m_callback;

public:

	explicit GenericCallback( const std::function<void()>& callback ) :
		m_callback( callback ) { }

	virtual ~GenericCallback( ) { }

	virtual CallResult call( )
	{
		if( m_callback )
		{
			m_callback( );

			return Success;
		}
		else
		{
			return Invalid;
		}
	}
};

void PostMessageToEventQueue( ros::CallbackQueue* pCallbackQueue,
		MessageProcessor& processor, std::vector<char> data )
{
	pCallbackQueue->addCallback( ros::CallbackInterfacePtr( new GenericCallback(
		std::bind( &MessageProcessor::ProcessMessage, &processor, data ) ) ) );
}

class BrainStem
{
private:

	ros::NodeHandle		m_node;

	ros::Timer			m_timer;

	bool				m_bIsSerialOpen;

	std::string			m_strSerialPort;

	SerialIO			m_serialIO;

	MessageProcessor	m_messageProcessor;

	float				m_fRetryTimeout;

public:

	BrainStem( const std::string& strSerialPort, float fRetryTimeout ) :
		m_node( ),
		m_timer( m_node.createTimer( ros::Duration( fRetryTimeout ),
			std::bind( &BrainStem::OnCheckSerialPort, this, std::placeholders::_1 ) ) ),
		m_bIsSerialOpen( false ),
		m_strSerialPort( strSerialPort ),
		m_serialIO( ),
		m_messageProcessor( m_node, &m_serialIO ),
		m_fRetryTimeout( fRetryTimeout )
	{
		CheckSerialPort( true );
	}

	~BrainStem( )
	{

	}

	void OnCheckSerialPort( const ros::TimerEvent& e )
	{
		CheckSerialPort( false );
	}

	void CheckSerialPort( bool bInitialCheck )
	{
		// Save current state (so we don't spam debug log on reconnect)
		bool bIsSerialOpen = m_bIsSerialOpen;

		m_bIsSerialOpen = m_serialIO.IsOpen( );

		std::string strError;

		if( !m_bIsSerialOpen )
		{
			try
			{
				ros::CallbackQueue* pCallbackQueue = ros::getGlobalCallbackQueue( );

				m_serialIO.Open( m_strSerialPort.c_str( ), std::bind( PostMessageToEventQueue,
					pCallbackQueue, m_messageProcessor, std::placeholders::_1 ) );

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

				ROS_ERROR( "Error connecting to serial port: %s (Retry: %.2fs)\n", m_fRetryTimeout, strError.c_str( ) );
			}
		}

		if( bIsSerialOpen != m_bIsSerialOpen )
		{
			if( m_bIsSerialOpen == true )
			{
				ROS_DEBUG( "Connected to serial port\n" );
			}
			else
			{
				ROS_ERROR( "Disconnected from serial port: %s (Retry: %.2fs)\n", m_fRetryTimeout, strError.c_str( ) );
			}
		}
	}
};

int main(int argc, char **argv)
{
	ROS_INFO( "Starting brain stem" );

	// Initialize ROS stuff
	ros::init( argc, argv, "brain_stem" );

	// Connect to serial port "/dev/malg" with a retry of 1s
	BrainStem brainStem ( "/dev/malg", 1.0f );

	ros::Rate rate( 100 );

	// Respond to inputs until shut down
	while( ros::ok( ) )
	{
		// Handle ROS events
		ros::spinOnce( );

		// Event loop rate
		rate.sleep( );
	}

	return 0;
}

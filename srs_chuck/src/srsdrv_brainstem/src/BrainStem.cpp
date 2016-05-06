#include <ros/ros.h>
#include <srslib_framework/utils/Thread.hpp>
#include <SerialIO.h>
#include <MessageProcessor.h>
#include <geometry_msgs/TwistStamped.h>

////////////////////////////////////////////////////////////////////////////////////////
// Main
////////////////////////////////////////////////////////////////////////////////////////

using namespace srs;

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
				// Anonymous function call the message processor in the main ros thread
				m_serialIO.Open( m_strSerialPort.c_str( ), [&](std::vector<char> buffer)
					{
						ExecuteInRosThread( std::bind( &MessageProcessor::ProcessMessage, &m_messageProcessor,
							buffer ) );
					} );

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
				ROS_ERROR( "Error connecting to serial port: %s (Retry: %.2f)\n", strError.c_str( ), m_fRetryTimeout );
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
				ROS_ERROR( "Disconnected from serial port: %s (Retry: %.2f)\n", strError.c_str( ), m_fRetryTimeout );
			}

			// Publish the connection updated message
			m_messageProcessor.SetConnected( m_bIsSerialOpen );
		}
	}
};

int main(int argc, char **argv)
{
   // Initialize ROS
    ros::init(argc, argv, "srsdrv_brainstem");

    ROS_INFO_STREAM("srsdrv_brainstem started");

	try
	{
		// Connect to serial port "/dev/malg" with a retry of 1s
		BrainStem brainStem ( "/dev/ttyUSB0", 1.0f );

		ros::Rate rate( 100 );

		// Respond to inputs until shut down
		while( ros::ok( ) )
		{
			// Handle ROS events
			ros::spinOnce( );

			// Event loop rate
			rate.sleep( );
		}
	}
    catch( ... )
    {
		ROS_DEBUG( "Error creating Brainstem\n" );
    }

	ROS_INFO( "Stopping brain stem" );

	return 0;
}

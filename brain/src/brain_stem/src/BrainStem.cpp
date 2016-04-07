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

int main(int argc, char **argv)
{
	ROS_INFO( "Starting brain stem" );

	ros::init( argc, argv, "brain_stem" );
	ros::NodeHandle node;

	ros::CallbackQueue* pCallbackQueue = ros::getGlobalCallbackQueue( );

	SerialIO serialIO;

	MessageProcessor messageProcessor( node, &serialIO );

	serialIO.Open( "/dev/malg", std::bind( PostMessageToEventQueue,
			pCallbackQueue, messageProcessor, std::placeholders::_1 ) );

	// Initialize ROS stuff

	// Respond to inputs until shut down
	ros::spin();

	return 0;
}

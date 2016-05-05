#include <StarGazerDriver.hpp>
#include <srslib_framework/Aps.h>

namespace srs
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
StarGazerDriver::StarGazerDriver( ) :
	rosNodeHandle_( ),
	rosApsPublisher( rosNodeHandle_.advertise<srslib_framework::Aps>( "/sensors/aps/pose", 1000 ) ),
	starGazer_( "/dev/ttyUSB0" )
{
	starGazer_.SetOdometryCallback(
		std::bind( &StarGazerDriver::OdometryCallback, this, std::placeholders::_1, std::placeholders::_2,
			std::placeholders::_3, std::placeholders::_4, std::placeholders::_5 ) );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void StarGazerDriver::run( )
{
	ros::Rate refreshRate( REFRESH_RATE_HZ );

	starGazer_.Configure( );
//	starGazer_.AutoCalculateHeight();
	starGazer_.Start( );

	while( ros::ok( ) )
	{
		ros::spinOnce( );

		starGazer_.PumpMessageProcessor( );

		refreshRate.sleep( );
	}

	starGazer_.Stop( );
}

void StarGazerDriver::OdometryCallback( int tagID, float x, float y, float z, float angle )
{
	srslib_framework::Aps msg;

	msg.x = x;
	msg.y = y;
	msg.yaw = angle;

	rosApsPublisher.publish( msg );

	ROS_DEBUG_NAMED( "StarGazer", "Tag: %04i (%2.2f, %2.2f, %2.2f) %2.2f deg\n", tagID, x, y, z, angle );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs

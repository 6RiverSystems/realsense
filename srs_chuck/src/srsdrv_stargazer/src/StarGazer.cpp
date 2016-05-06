/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <StarGazer.h>
#include <srslib_framework/Aps.h>

namespace srs
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

// TODO: Pass in serial port and #define
////////////////////////////////////////////////////////////////////////////////////////////////////

StarGazer::StarGazer( const std::string& strSerialPort, const std::string& strApsTopic ) :
	rosNodeHandle_( ),
	rosApsPublisher( rosNodeHandle_.advertise<srslib_framework::Aps>( strApsTopic, 1000 ) ),
	starGazerDriver_( strSerialPort )
{
	starGazerDriver_.SetOdometryCallback(
		std::bind( &StarGazer::OdometryCallback, this, std::placeholders::_1, std::placeholders::_2,
			std::placeholders::_3, std::placeholders::_4, std::placeholders::_5 ) );
}

StarGazer::~StarGazer( )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void StarGazer::run( )
{
	ros::Rate refreshRate( REFRESH_RATE_HZ );

	starGazerDriver_.Configure( );

	// TODO: Set calculate height

//	StarGazer_.AutoCalculateHeight();

	starGazerDriver_.Start( );

	while( ros::ok( ) )
	{
		ros::spinOnce( );

		starGazerDriver_.PumpMessageProcessor( );

		refreshRate.sleep( );
	}

	starGazerDriver_.Stop( );
}

void StarGazer::OdometryCallback( int nTagId, float fX, float fY, float fZ, float fAngle )
{
	// TODO: Publish tag id (and z)
	srslib_framework::Aps msg;

	msg.tagId = nTagId;
	msg.x = fX;
	msg.y = fY;
	msg.z = fZ;
	msg.yaw = fAngle;

	rosApsPublisher.publish( msg );

	ROS_DEBUG_NAMED( "StarGazer", "Tag: %04i (%2.2f, %2.2f, %2.2f) %2.2f deg\n", nTagId, fX, fY, fZ, fAngle );
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs

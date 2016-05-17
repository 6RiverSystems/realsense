/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <StarGazer.h>
#include <srslib_framework/Aps.h>
#include <srslib_framework/io/SerialIO.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <boost/lexical_cast.hpp>

namespace YAML {

template<>
struct convert<srs::Anchor>
{
    static bool decode(const Node& node, srs::Anchor& anchor)
    {
        anchor.id = node["id"].as<std::string>();
        anchor.x = node["location"][0].as<int>();
        anchor.y = node["location"][1].as<int>();
        anchor.z = node["location"][2].as<int>();
        anchor.orientation = node["orientation"].as<int>();

        return true;
    }
};

} // YAML

namespace srs
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

// TODO: Pass in serial port and #define
////////////////////////////////////////////////////////////////////////////////////////////////////

StarGazer::StarGazer( const std::string& strSerialPort, const std::string& strApsTopic ) :
	m_rosNodeHandle( ),
	m_rosApsPublisher( m_rosNodeHandle.advertise<srslib_framework::Aps>( strApsTopic, 1000 ) ),
	m_pSerialIO( new SerialIO( ) ),
	m_messageProcessor( m_pSerialIO )
{
	LoadAnchors( );

	m_messageProcessor.SetOdometryCallback(
		std::bind( &StarGazer::OdometryCallback, this, std::placeholders::_1, std::placeholders::_2,
			std::placeholders::_3, std::placeholders::_4, std::placeholders::_5 ) );

	m_messageProcessor.SetReadCallback(
		std::bind( &StarGazer::ReadCallback, this, std::placeholders::_1, std::placeholders::_2 ) );


	std::shared_ptr<SerialIO> pSerialIO = std::dynamic_pointer_cast<SerialIO>( m_pSerialIO );

	pSerialIO->SetLeadingCharacter( STARGAZER_STX );
	pSerialIO->SetTerminatingCharacter( STARGAZER_RTX );
	pSerialIO->SetFirstByteDelay( std::chrono::microseconds( 30000 ) );
	pSerialIO->SetByteDelay( std::chrono::microseconds( 5000 ) );

	pSerialIO->Open( strSerialPort.c_str( ), std::bind( &StarGazerMessageProcessor::ProcessStarGazerMessage,
		&m_messageProcessor, std::placeholders::_1 ) );
}

StarGazer::~StarGazer( )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void StarGazer::Run( )
{
	ros::Rate refreshRate( REFRESH_RATE_HZ );

	Configure( );

	Start( );

	while( ros::ok( ) )
	{
		ros::spinOnce( );

		PumpMessageProcessor( );

		refreshRate.sleep( );
	}

	Stop( );
}


void StarGazer::HardReset( )
{
	Stop( );

	m_messageProcessor.HardReset( );
}

void StarGazer::Configure( )
{
	Stop( );

	m_messageProcessor.GetVersion( );

	m_messageProcessor.SetMarkType( STAR_GAZER_LANDMARK_TYPES::HLD3L );

	m_messageProcessor.SetEnd( );
}

void StarGazer::SetFixedHeight( int height_mm )
{
	Stop( );
	m_messageProcessor.SetMarkHeight( height_mm );
	m_messageProcessor.HightFix( true );
	m_messageProcessor.SetEnd( );
}

void StarGazer::SetVariableHeight( )
{
	Stop( );
	m_messageProcessor.HightFix( false );
	m_messageProcessor.SetEnd( );
}

void StarGazer::AutoCalculateHeight( )
{
	Stop( );
	m_messageProcessor.HeightCalc( );
}

void StarGazer::Start( )
{
	m_messageProcessor.CalcStart( );
}

void StarGazer::Stop( )
{
	m_messageProcessor.CalcStop( );
}

void StarGazer::PumpMessageProcessor( )
{
	m_messageProcessor.PumpMessageProcessor( );
}

void StarGazer::ReadCallback( std::string strType, std::string strValue )
{
	ROS_DEBUG_STREAM_NAMED( "StarGazer", strType << " = " << strValue );
}

void StarGazer::OdometryCallback( int nTagId, float fX, float fY, float fZ, float fAngle )
{
	auto iter = m_mapAnchorTransforms.find( nTagId );

	if( iter != m_mapAnchorTransforms.end( ) )
	{
		// The anchor transform from the origin of the map to the anchor point
		const tf::Transform& anchorGlobal = iter->second;

		tf::Vector3 point( fX, fY, fZ );

		tf::Quaternion orientation;
		orientation.setRPY( 0.0, 0.0, fAngle );

		// Transform the incoming point based to the global coordinate system
		tf::Vector3 transformedPoint = anchorGlobal * point;
		tf::Quaternion transformedOrientation = anchorGlobal * orientation;

		srslib_framework::Aps msg;

		msg.tagId = nTagId;
		msg.x = transformedPoint.getX( );
		msg.y = transformedPoint.getY( );
		msg.z = transformedPoint.getZ( );
		msg.yaw = orientation.getAngle( );

		m_rosApsPublisher.publish( msg );

		tf::Vector3 tagPoint = anchorGlobal.getOrigin( );
		tf::Quaternion tagOrientation = anchorGlobal.getRotation( );

		ROS_DEBUG_NAMED( "StarGazer", "Tag Location: %04i (%2.2f, %2.2f, %2.2f) %2.2f deg\n",
			nTagId, tagPoint.getX( ), tagPoint.getY( ), tagPoint.getZ( ), tagOrientation.getAngle( ) );

		ROS_DEBUG_NAMED( "StarGazer", "StarGazer Location: %04i (%2.2f, %2.2f, %2.2f) %2.2f deg\n",
			nTagId, fX, fY, fZ, fAngle );

		ROS_DEBUG_NAMED( "StarGazer", "Global Location: %04i (%2.2f, %2.2f, %2.2f) %2.2f deg\n",
			nTagId, msg.x, msg.y, msg.z, msg.yaw );
	}
	else
	{
		ROS_DEBUG_NAMED( "StarGazer", "Invalid or Unknown Tag: %04i (%2.2f, %2.2f, %2.2f) %2.2f deg\n",
			nTagId, fX, fY, fZ, fAngle );
	}
}

void StarGazer::LoadAnchors( )
{
	string anchorsFilename;
	m_rosNodeHandle.param( "target_anchors", anchorsFilename, string( "" ) );

	ROS_INFO_STREAM( "Loading anchors from " << anchorsFilename );

	YAML::Node document = YAML::LoadFile( anchorsFilename );

	if( !document.IsNull( ) )
	{
		vector<Anchor> anchors = document["anchors"].as<vector<Anchor>>( );

		for( auto anchor : anchors )
		{
			try
			{
				int32_t anchorId = boost::lexical_cast<int32_t>( anchor.id );

				tf::Transform transform;

				tf::Vector3 origin( anchor.x, anchor.y, anchor.z );
				transform.setOrigin( origin );

				tf::Quaternion orientation;
				orientation.setRPY( 0.0, 0.0, anchor.orientation );
				transform.setRotation( orientation );

				ROS_INFO_STREAM( "Anchor: id=" << anchor.id << ", x=" << anchor.x <<
					", y=" << anchor.y << ", z=" << anchor.z << ", orientation=" << anchor.orientation );

				m_mapAnchorTransforms[anchorId] = transform;
			}
			catch( const boost::bad_lexical_cast& e )
			{
				ROS_INFO_STREAM( "Could not convert anchor id to int: " << anchor.id << " " << e.what( ) );
			}
		}
	}
	else
	{
		ROS_ERROR_STREAM( "Configuration file not found: " << anchorsFilename );
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs

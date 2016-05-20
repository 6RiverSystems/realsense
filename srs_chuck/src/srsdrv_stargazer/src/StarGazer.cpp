/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <StarGazer.h>
#include <srslib_framework/Aps.h>
#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <boost/lexical_cast.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>

namespace YAML {

template<>
struct convert<srs::Anchor>
{
    static bool decode(const Node& node, srs::Anchor& anchor)
    {
        anchor.id = node["id"].as<std::string>();
        anchor.x = node["location"][0].as<double>();
        anchor.y = node["location"][1].as<double>();
        anchor.z = node["location"][2].as<double>();
        anchor.orientation = node["orientation"].as<double>();

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

StarGazer::StarGazer( const std::string& strNodeName, const std::string& strSerialPort,
	const std::string& strApsTopic ) :
	m_rosNodeHandle( strNodeName ),
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
	pSerialIO->SetByteDelay( std::chrono::microseconds( 2000 ) );

	auto processMessage = [&]( std::vector<char> buffer )
		{
			ExecuteInRosThread( std::bind( &StarGazerMessageProcessor::ProcessStarGazerMessage, &m_messageProcessor,
				buffer ) );
		};

	auto connectionChanged = [&]( bool bIsConnected )
	{
		ExecuteInRosThread( std::bind( &StarGazer::OnConnectionChanged, this,
				bIsConnected ) );
	};

	pSerialIO->Open( strSerialPort.c_str( ), connectionChanged, processMessage );
}

StarGazer::~StarGazer( )
{

}

////////////////////////////////////////////////////////////////////////////////////////////////////
void StarGazer::Run( )
{
	ros::Rate refreshRate( REFRESH_RATE_HZ );

	while( ros::ok( ) )
	{
		ros::spinOnce( );

		PumpMessageProcessor( );

		refreshRate.sleep( );
	}
}


void StarGazer::OnConnectionChanged( bool bIsConnected )
{
	ROS_DEBUG_STREAM_NAMED( "StarGazer", "Connected" );

	if( bIsConnected )
	{
		Configure( );

		Start( );
	}
}

void StarGazer::HardReset( )
{
	Stop( );

	m_messageProcessor.HardReset( );
}

void StarGazer::Configure( )
{
	m_messageProcessor.ResetState( );

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

using namespace boost::accumulators;

typedef accumulator_set<double, stats<tag::median > > double_acc;

void StarGazer::OdometryCallback( int nTagId, float fX, float fY, float fZ, float fAngle )
{
	auto iter = m_mapAnchorTransforms.find( nTagId );

	if( iter != m_mapAnchorTransforms.end( ) )
	{
		// The anchor transform from the origin of the map to the anchor point
		const tf::Transform& anchorGlobal = iter->second;

		tf::Vector3 anchorOrigin = anchorGlobal.getOrigin( );
		tf::Quaternion anchorRotation = anchorGlobal.getRotation( );

		if( fAngle < 0 )
		{
			fAngle += 360.0f;
		}

		double fAngleInRadians = fAngle * M_PI / 180.0f;

		double dfOffset = 14.0f;

		fX -= (dfOffset * cos( fAngleInRadians ));
		fY += (dfOffset * sin( fAngleInRadians ));

		tf::Vector3 point( fX, fY, fZ );
		tf::Quaternion orientation = tf::createQuaternionFromYaw( fAngleInRadians );

		tf::Vector3 transformedPoint = anchorGlobal * point;
		tf::Quaternion transformedOrientation = anchorGlobal * orientation;

		srslib_framework::Aps msg;

		msg.tagId = nTagId;
		msg.x = transformedPoint.getX( ) * 10.0f;
		msg.y = transformedPoint.getY( ) * 10.0f;
		msg.z = transformedPoint.getZ( ) * 10.0f;
		msg.yaw = transformedOrientation.getAngle( );

		m_rosApsPublisher.publish( msg );

		static uint32_t sCount = 0;

		static double_acc accLocalX;
		static double_acc accLocalY;
		static double_acc accLocalZ;
		static double_acc accLocalRotation;

		static double_acc accGlobalX;
		static double_acc accGlobalY;
		static double_acc accGlobalZ;
		static double_acc accGlobalRotation;

		accLocalX( point.getX( ) );
		accLocalY( point.getY( ) );
		accLocalZ( point.getZ( ) );
		accLocalRotation( orientation.getAngle( ) );

		accGlobalX( msg.x );
		accGlobalY( msg.y );
		accGlobalZ( msg.z );
		accGlobalRotation( msg.yaw );

		if( sCount++ == 50 )
		{
			ROS_DEBUG_NAMED( "StarGazer", "Anchor Location (median): %04i (%2.6f, %2.6f, %2.6f) %2.6f rad\n",
				nTagId, anchorOrigin.getX( ), anchorOrigin.getY( ), anchorOrigin.getZ( ), anchorRotation.getAngle( ) );

			ROS_DEBUG_NAMED( "StarGazer", "%04i, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f",
				nTagId, median( accLocalX ), median( accLocalY ), median( accLocalZ ), median( accLocalRotation ),
				median( accGlobalX ), median( accGlobalY ), median( accGlobalZ ), median( accGlobalRotation ) );

			// reset accumulators
			accLocalX = double_acc( );
			accLocalY = double_acc( );
			accLocalZ = double_acc( );
			accLocalRotation = double_acc( );

			accGlobalX = double_acc( );
			accGlobalY = double_acc( );
			accGlobalZ = double_acc( );
			accGlobalRotation = double_acc( );

			sCount = 0;
		}
	}
	else
	{
		ROS_INFO_NAMED( "StarGazer", "Invalid or Unknown Tag: %04i (%2.2f, %2.2f, %2.2f) %2.2f rad\n",
			nTagId, fX, fY, fZ, fAngle );
	}
}

void StarGazer::LoadAnchors( )
{
	string anchorsFilename;
	m_rosNodeHandle.param( "target_anchors", anchorsFilename, string( "" ) );

	ROS_INFO_STREAM( "Loading anchors from " << anchorsFilename );

	try
	{
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

//					tf::Pose worldPose;
//
//					tf::Pose anchorPose;
//
//					tf::Transform transform = worldPose.inverseTimes( anchorPose );

					tf::Vector3 origin( anchor.x, anchor.y, -anchor.z );
					transform.setOrigin( origin );

					tf::Quaternion orientation = tf::createQuaternionFromYaw( anchor.orientation );

					// Convert left hand rule of stargazer to right hand rule (ROS coordinate system)
					tf::Quaternion rightHandOrientation = tf::Quaternion( orientation.getX( ), orientation.getY( ),
						-orientation.getZ( ), -orientation.getW( ) );

					transform.setRotation( rightHandOrientation );

// Test changing left to right hand rule
//					for( int i = 0; i < 360; i++ )
//					{
//						tf::Quaternion orientationTest = tf::createQuaternionFromYaw( (double)i * M_PI / 180.0f );
//
//						orientationTest = orientationTest * orientation;
//
//						ROS_INFO( "Angle: %d = %f", i, orientationTest.getAngle( ) * 180.0f / M_PI );
//					}

					ROS_INFO_STREAM( "Anchor: id=" << anchor.id << ", x=" << anchor.x <<
						", y=" << anchor.y << ", z=" << anchor.z << ", orientation=" << rightHandOrientation.getAngle( ) );

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
	catch( const std::runtime_error& e )
	{
		ROS_INFO_STREAM( "Could not parse yaml file for anchors: " << anchorsFilename << " " << e.what( ) );
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs

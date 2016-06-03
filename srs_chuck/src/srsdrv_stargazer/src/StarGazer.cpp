/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <StarGazer.h>
#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <iostream>
#include <boost/lexical_cast.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/PoseStamped.h>

namespace YAML
{

template<>
struct convert<srs::Anchor>
{
	static bool decode( const Node& node, srs::Anchor& anchor )
	{
		anchor.id = node["id"].as<std::string>( );
		anchor.x = node["location"][0].as<double>( );
		anchor.y = node["location"][1].as<double>( );
		anchor.z = node["location"][2].as<double>( );
		anchor.orientation = node["orientation"].as<double>( );

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

StarGazer::StarGazer( const std::string& strNodeName, const std::string& strSerialPort, const std::string& strApsTopic ) :
	m_rosNodeHandle( strNodeName ),
	m_rosApsPublisher( m_rosNodeHandle.advertise < geometry_msgs::PoseStamped > (strApsTopic, 1000) ),
	m_pSerialIO( new SerialIO( ) ),
	m_messageProcessor( m_pSerialIO ),
	m_filter( 2.6, 5.0, 70 ),
	m_strAnchorTargetFrame( ),
	m_strPoseTargetFrame( ),
	m_sleeper( REFRESH_RATE_HZ / 1000.0 ),
	m_mapTransformSenders( ),
	m_listener( ),
	m_baseFootprintTransform( ),
	m_rotationTransform( tf::createQuaternionFromRPY( M_PI, 0.0f, 0.0 ) )
{
	LoadAnchors( );

	m_messageProcessor.SetOdometryCallback(
		std::bind( &StarGazer::OdometryCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
			std::placeholders::_4, std::placeholders::_5 ) );

	m_messageProcessor.SetReadCallback( std::bind( &StarGazer::ReadCallback, this, std::placeholders::_1, std::placeholders::_2 ) );

	std::shared_ptr<SerialIO> pSerialIO = std::dynamic_pointer_cast < SerialIO > (m_pSerialIO);

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

typedef accumulator_set<double, stats<tag::median> > double_acc;

void StarGazer::OdometryCallback( int nTagId, float fX, float fY, float fZ, float fAngle )
{
	ros::Time now( ros::Time::now( ) );

	auto iter = m_mapTransforms.find( nTagId );

	if( iter != m_mapTransforms.end( ) )
	{
		if( m_filter.unfilterStargazerData( nTagId, fX, fY, fZ ) )
		{
			std::string strAnchorFrame = GetAnchorFrame( nTagId );

			// Transform to map coordinate system (right hand rule)
			tf::Quaternion anchorRotation( m_rotationTransform * tf::createQuaternionFromYaw( fAngle * M_PI / 180.0f ) );

			// Add the stargazer orientation
			tf::Quaternion mapRotation( tf::createQuaternionFromYaw(
				tf::getYaw( anchorRotation ) + tf::getYaw( iter->second.getRotation( ) ) ) );

			// Rotated around map origin
			tf::Pose poseFootprintRotated( mapRotation );

			// Foot print offset is the footprint transform around the current rotation
			tf::Pose poseFootprint = poseFootprintRotated * m_baseFootprintTransform;

			// Offset in non rotated space from the anchor
			tf::Pose poseStargazerRotated( iter->second.getRotation( ) );
			tf::Vector3 stargazerOffset( fX / 100.0f, fY / 100.0f, fZ / 100.0f );
			tf::Transform stargazerTranslation( tf::Quaternion::getIdentity( ), stargazerOffset );

			tf::Pose poseStargazer = poseStargazerRotated * stargazerTranslation;

			tf::Vector3 mapOffset = iter->second.getOrigin( );

			tf::Vector3 stargazerOffsetMap = poseStargazer.getOrigin( );

			tf::Vector3 footprintOffsetMap = poseFootprint.getOrigin( );

			// Transform to map
			tf::Pose poseMap( mapRotation, mapOffset + stargazerOffsetMap + footprintOffsetMap );

			std::ostringstream stream;
			stream << "Stargazer Data: " <<
				tf::getYaw( anchorRotation ) * 180.0f / M_PI << ", " <<
				tf::getYaw( mapRotation ) * 180.0f / M_PI << ", " <<
				mapOffset.getX( ) << ", " << mapOffset.getY( ) << ", " << mapOffset.getZ( ) << ", " <<
				stargazerOffset.getX( ) << ", " << stargazerOffset.getY( ) << ", " << stargazerOffset.getZ( ) << ", " <<
				stargazerOffsetMap.getX( ) << ", " << stargazerOffsetMap.getY( ) << ", " << stargazerOffsetMap.getZ( ) << ", " <<
				footprintOffsetMap.getX( ) << ", " << footprintOffsetMap.getY( ) << ", " << footprintOffsetMap.getZ( ) << ", " <<
				poseMap.getOrigin( ).getX( ) << ", " << poseMap.getOrigin( ).getY( ) << ", " <<
				poseMap.getOrigin( ).getZ( ) << ", " << tf::getYaw( poseMap.getRotation( ) ) * 180 / M_PI << std::endl;

			std::string strData =  stream.str( );

			ROS_DEBUG_NAMED( "StarGazer", "%s", strData.c_str( ) );

			tf::Stamped<tf::Pose> stampedPose( poseMap, now, m_strPoseTargetFrame );

			// The data from the anchor frame reference
			geometry_msgs::PoseStamped msg;
			poseStampedTFToMsg( stampedPose, msg );

			m_rosApsPublisher.publish( msg );

			static double_acc accLocalX;
			static double_acc accLocalY;
			static double_acc accLocalZ;
			static double_acc accLocalRotation;

			static double_acc accGlobalX;
			static double_acc accGlobalY;
			static double_acc accGlobalZ;
			static double_acc accGlobalRotation;

			accLocalX( fX );
			accLocalY( fY );
			accLocalZ( fZ );
			accLocalRotation( fAngle );

			accGlobalX( msg.pose.position.x );
			accGlobalY( msg.pose.position.y );
			accGlobalZ( msg.pose.position.z );
			accGlobalRotation( tf::getYaw( msg.pose.orientation ) );

			static int sCount = 0;
			if( sCount++ == 50 )
			{
				//			ROS_DEBUG_NAMED( "StarGazer", "Anchor Location (median): %04i (%2.6f, %2.6f, %2.6f) %2.6f rad\n",
				//				nTagId, anchorOrigin.getX( ), anchorOrigin.getY( ), anchorOrigin.getZ( ), anchorRotation.getAngle( ) );
				//
				//			ROS_DEBUG_NAMED( "StarGazer", "%04i, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f, %2.6f",
				//				nTagId, median( accLocalX ), median( accLocalY ), median( accLocalZ ), median( accLocalRotation ),
				//				median( accGlobalX ), median( accGlobalY ), median( accGlobalZ ), median( accGlobalRotation ) );

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
			ROS_DEBUG_NAMED( "StarGazer", "Rejected anchor position: %04i (%2.2f, %2.2f, %2.2f) %2.2f degrees\n", nTagId, fX, fY, fZ,
				fAngle );
		}
	}
	else
	{
		m_filter.reportDeadZone( );

		ROS_DEBUG_NAMED( "StarGazer", "Invalid or Unknown Tag: %04i (%2.2f, %2.2f, %2.2f) %2.2f degrees\n", nTagId, fX, fY, fZ, fAngle );
	}
}

void StarGazer::LoadAnchors( )
{
	try
	{
		ros::Time now = ros::Time( 0 );

		std::string strStargazerOptical( "/stargazer_optical_frame" );
		std::string strFootprint( "/base_footprint" );

		m_listener.waitForTransform( strStargazerOptical, strFootprint, now, ros::Duration( 10.0 ) );
		m_listener.lookupTransform( strStargazerOptical, strFootprint, now, m_baseFootprintTransform );

		tf::Vector3 origin = m_baseFootprintTransform.getOrigin( );
		tf::Quaternion rotation = m_baseFootprintTransform.getRotation( );

		ROS_INFO_STREAM( "/stargazer_optical_frame -> /base_footprint: " << "x=" << origin.getX( ) <<
			", y=" << origin.getY( ) << ", z=" << origin.getZ( ) <<
			", orientation=" << rotation.getAngle( ) );
	}
	catch( const tf::TransformException& ex )
	{
		ROS_ERROR( "%s", ex.what( ) );
	}

	string anchorsFilename;
	m_rosNodeHandle.param( "target_anchors", anchorsFilename, string( "" ) );

	ROS_INFO_STREAM( "Loading anchors from " << anchorsFilename );

	try
	{
		YAML::Node document = YAML::LoadFile( anchorsFilename );

		if( !document.IsNull( ) )
		{
			m_rosNodeHandle.param( "target_frame", m_strPoseTargetFrame, string( "" ) );

			if( !m_strPoseTargetFrame.length( ) )
			{
				m_strPoseTargetFrame = "/map";

				ROS_ERROR_STREAM( "Stargazer *pose* target frame not set, defaulting to \"" << m_strPoseTargetFrame << "\"" );
			}
			else
			{
				ROS_INFO_STREAM( "Stargazer *pose* target frame: " << m_strPoseTargetFrame );
			}

			vector<Anchor> anchors = document["anchors"].as<vector<Anchor>>( );
			m_strAnchorTargetFrame = document["target_frame"].as<std::string>( );

			if( !m_strAnchorTargetFrame.length( ) )
			{
				m_strAnchorTargetFrame = "/map";

				ROS_ERROR_STREAM( "Stargazer *anchor* target frame not set, defaulting to \"" << m_strAnchorTargetFrame << "\"" );
			}
			else
			{
				ROS_INFO_STREAM( "Stargazer *anchor* target frame: " << m_strAnchorTargetFrame );
			}

			for( auto anchor : anchors )
			{
				try
				{
					int32_t anchorId = boost::lexical_cast < int32_t > (anchor.id);

					tf::Vector3 origin( anchor.x, anchor.y, -anchor.z );
					tf::Quaternion orientation = tf::createQuaternionFromYaw( anchor.orientation );

					tf::Transform transform( orientation, origin );

					ROS_INFO_STREAM( "Anchor: id=" << anchor.id << ", x=" << anchor.x <<
						", y=" << anchor.y << ", z=" << anchor.z << ", orientation=" << tf::getYaw( orientation ) * 180 / M_PI );

					m_mapTransforms[anchorId] = transform;

					std::string strAnchorFrame = GetAnchorFrame( anchorId );

					TransformBroadcasterPtr oAnchorTransformSender( new tf2_ros::StaticTransformBroadcaster( ) );

					geometry_msgs::TransformStamped staticTransformStamped;
					staticTransformStamped.header.stamp = ros::Time( 0 );
					staticTransformStamped.header.frame_id = m_strAnchorTargetFrame;
					staticTransformStamped.child_frame_id = strAnchorFrame;
					staticTransformStamped.transform.translation.x = origin.getX( );
					staticTransformStamped.transform.translation.y = origin.getY( );
					staticTransformStamped.transform.translation.z = origin.getZ( );
					staticTransformStamped.transform.rotation.x = orientation.getX( );
					staticTransformStamped.transform.rotation.y = orientation.getY( );
					staticTransformStamped.transform.rotation.z = orientation.getZ( );
					staticTransformStamped.transform.rotation.w = orientation.getW( );

					ROS_INFO_STREAM( "Anchor: " << "id=" << anchor.id <<
						", frame_id=" << staticTransformStamped.header.frame_id <<
						", child_frame_id=" << staticTransformStamped.child_frame_id <<
						", x=" << staticTransformStamped.transform.translation.x <<
						", y=" << staticTransformStamped.transform.translation.y <<
						", z=" << staticTransformStamped.transform.translation.z <<
						", orientation=" << tf::getYaw( staticTransformStamped.transform.rotation ) * 180 / M_PI );

					// Send the static transform message (only necessary one time)
					oAnchorTransformSender->sendTransform( staticTransformStamped );

					m_mapTransformSenders[anchorId] = oAnchorTransformSender;
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

std::string StarGazer::GetAnchorFrame( int tagID ) const
{
	std::ostringstream stringStream;
	stringStream << "/aps_anchor_";
	stringStream << tagID;

	return std::move( stringStream.str( ) );
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs

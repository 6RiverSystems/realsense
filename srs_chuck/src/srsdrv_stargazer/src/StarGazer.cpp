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
	m_stargazerTransform( ),
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
			// The anchor transform from the origin of the map to the anchor point
			const tf::Transform& anchorGlobal = iter->second;

			std::string strAnchorFrame = GetAnchorFrame( nTagId );

			double dfAngleInRadians = fAngle * M_PI / 180.0f;

			// Transform to map coordinate system (right hand rule)
			tf::Quaternion anchorRotation( m_rotationTransform * tf::createQuaternionFromYaw( dfAngleInRadians ) );

			tf::Vector3 stargazerOffset( fX / 100.0f, fY / 100.0f, fZ / 100.0f );

			// Rotated around map origin
			tf::Pose poseFootprintRotated( anchorRotation, stargazerOffset );

			// Translate by optical -> camera
			tf::Pose poseCamera = poseFootprintRotated * m_stargazerTransform;

			tf::Vector3 cameraOrigin = poseCamera.getOrigin( );

			// Translate by camera -> base footprint
			tf::Pose poseBaseFootprint = poseCamera * m_baseFootprintTransform;

			tf::Vector3 footprintOrigin = poseBaseFootprint.getOrigin( );

			tf::Vector3 chuckOrigin = anchorGlobal * footprintOrigin;

			tf::Quaternion chuckOrientation = anchorGlobal * anchorRotation;

			tf::Pose poseMap( chuckOrientation, chuckOrigin);
//
//			std::ostringstream stream;
//
//			stream << "Stargazer Data: " << std::fixed <<  endl <<
//				tf::getYaw( anchorRotation ) * 180.0f / M_PI << ", " <<
//				stargazerOffset.getX( ) << ", " << stargazerOffset.getY( ) << ", " << stargazerOffset.getZ( ) << ", " <<
//				cameraOrigin.getX( ) << ", " << cameraOrigin.getY( ) << ", " << cameraOrigin.getZ( ) << ", " <<
//				footprintOrigin.getX( ) << ", " << footprintOrigin.getY( ) << ", " << footprintOrigin.getZ( ) << ", " <<
//				chuckOrigin.getX( ) << ", " << chuckOrigin.getY( ) << ", " << chuckOrigin.getZ( ) << ", " <<
//				tf::getYaw( chuckOrientation ) * 180.0f / M_PI << ", " << endl;
//
//			std::string strData =  stream.str( );
//
//			ROS_DEBUG_NAMED( "StarGazer", "%s", strData.c_str( ) );

			tf::Stamped<tf::Pose> stampedPose( poseMap, now, m_strPoseTargetFrame );

			// The data from the anchor frame reference
			geometry_msgs::PoseStamped msg;
			poseStampedTFToMsg( stampedPose, msg );

			m_rosApsPublisher.publish( msg );

			static double_acc accLocalX;
			static double_acc accLocalY;
			static double_acc accLocalZ;
			static double_acc accLocalRotation;

			static double_acc accCameraX;
			static double_acc accCameraY;
			static double_acc accCameraZ;
			static double_acc accCameraRotation;

			static double_acc accFootprintX;
			static double_acc accFootprintY;
			static double_acc accFootprintZ;

			static double_acc accChuckX;
			static double_acc accChuckY;
			static double_acc accChuckZ;
			static double_acc accChuckRotation;

			accLocalX( stargazerOffset.getX( ) );
			accLocalY( stargazerOffset.getY( ) );
			accLocalZ( stargazerOffset.getZ( ) );

			accCameraX( cameraOrigin.getX( ) );
			accCameraY( cameraOrigin.getY( ) );
			accCameraZ( cameraOrigin.getZ( ) );

			accFootprintX( footprintOrigin.getX( ) );
			accFootprintY( footprintOrigin.getY( ) );
			accFootprintZ( footprintOrigin.getZ( ) );

			accChuckX( msg.pose.position.x );
			accChuckY( msg.pose.position.y );
			accChuckZ( msg.pose.position.z );
			accChuckRotation( tf::getYaw( msg.pose.orientation ) * 180.0f / M_PI );

			static int sCount = 0;
			if( sCount++ == 50 )
			{
				std::ostringstream stream;

				stream << "Stargazer Data: " << std::fixed <<  endl <<
					tf::getYaw( anchorRotation ) * 180.0f / M_PI << ", " <<
					median( accLocalX ) << ", " << median( accLocalY ) << ", " << median( accLocalZ ) << ", " <<
					median( accCameraX ) << ", " << median( accCameraY ) << ", " << median( accCameraZ ) << ", " <<
					median( accFootprintX ) << ", " << median( accFootprintY ) << ", " << median( accFootprintZ ) << ", " <<
					median( accChuckX ) << ", " << median( accChuckY ) << ", " << median( accChuckZ ) << ", " <<
					median( accChuckRotation ) << endl;

				std::string strData =  stream.str( );

				ROS_ERROR_NAMED( "StarGazer", "%s", strData.c_str( ) );

				// reset accumulators
				accLocalX = double_acc( );
				accLocalY = double_acc( );
				accLocalZ = double_acc( );

				accCameraX = double_acc( );
				accCameraY = double_acc( );
				accCameraZ = double_acc( );

				accFootprintX = double_acc( );
				accFootprintY = double_acc( );
				accFootprintZ = double_acc( );

				accChuckX = double_acc( );
				accChuckY = double_acc( );
				accChuckZ = double_acc( );
				accChuckRotation = double_acc( );

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
		std::string strStargazerLink( "/stargazer_camera_frame" );
		std::string strFootprint( "/base_footprint" );

		m_listener.waitForTransform( strStargazerOptical, strStargazerLink, now, ros::Duration( 10.0 ) );
		m_listener.lookupTransform( strStargazerOptical, strStargazerLink, now, m_stargazerTransform );

		m_listener.waitForTransform( strStargazerLink, strFootprint, now, ros::Duration( 10.0 ) );
		m_listener.lookupTransform( strStargazerLink, strFootprint, now, m_baseFootprintTransform );

		tf::Vector3 originStargazer = m_stargazerTransform.getOrigin( );
		tf::Quaternion rotationStargazer = m_stargazerTransform.getRotation( );

		ROS_INFO_STREAM( "/stargazer_optical_frame -> /stargazer_camera_frame: " << "x=" << originStargazer.getX( ) <<
			", y=" << originStargazer.getY( ) << ", z=" << originStargazer.getZ( ) <<
			", orientation=" << tf::getYaw( rotationStargazer ) );

		// TODO: Load from configuration file
		tf::Vector3 calculatedOrigin( 0.3087f, -0.032f, 0.0f );
		m_baseFootprintTransform.setOrigin( calculatedOrigin );
		tf::Vector3 originFootprint = m_baseFootprintTransform.getOrigin( );
		tf::Quaternion rotationFootprint = m_baseFootprintTransform.getRotation( );

		ROS_INFO_STREAM( "/stargazer_camera_frame -> /base_footprint: " << "x=" << originFootprint.getX( ) <<
			", y=" << originFootprint.getY( ) << ", z=" << originFootprint.getZ( ) <<
			", orientation=" << tf::getYaw( rotationFootprint ) );
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

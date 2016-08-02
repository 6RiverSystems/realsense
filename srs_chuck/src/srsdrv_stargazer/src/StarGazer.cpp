/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <StarGazer.h>
#include <srslib_framework/io/SerialIO.hpp>
#include <srslib_framework/platform/Thread.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <string>
#include <iostream>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <yaml-cpp/yaml.h>

#include <srslib_framework/MsgPose.h>

namespace srs
{

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

// TODO: Pass in serial port and #define
////////////////////////////////////////////////////////////////////////////////////////////////////

StarGazer::StarGazer( const std::string& strNodeName, const std::string& strSerialPort, const std::string& strApsTopic ) :
	m_rosNodeHandle( strNodeName ),
    m_rosApsPublisher(m_rosNodeHandle.advertise<srslib_framework::MsgPose>(strApsTopic, 1000)),
	m_pSerialIO( new SerialIO( "stargazer" ) ),
	m_messageProcessor( m_pSerialIO ),
	m_sleeper( REFRESH_RATE_HZ / 1000.0 ),
	m_mapTransformSenders( ),
	m_listener( ),
	m_pointTransformer( )
{
	LoadTransforms( );

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
	ROS_DEBUG_STREAM( "Stargazer: Connected" );

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
	ROS_DEBUG_STREAM( "Stargazer: " << strType << " = " << strValue );
}

void StarGazer::OdometryCallback( int nTagId, float fX, float fY, float fZ, float fAngle )
{
    tf::Pose pose(tf::Quaternion::getIdentity());

    if (m_pointTransformer.TransformPoint(nTagId, fX, fY, fZ, fAngle, pose))
    {
        tf::Stamped<tf::Pose> stampedPose(pose, ros::Time::now(),
            m_pointTransformer.GetAnchorFrame(nTagId));

        // The data from the anchor frame reference
        geometry_msgs::PoseStamped msg;
        poseStampedTFToMsg(stampedPose, msg);

        // Create our internal message
        srslib_framework::MsgPose message;

        message.header = msg.header;
        message.x = msg.pose.position.x;
        message.y = msg.pose.position.y;
        message.theta = AngleMath::rad2deg<double>(tf::getYaw(msg.pose.orientation));

        m_rosApsPublisher.publish(message);
    }
}

void StarGazer::LoadTransforms( )
{
	std::string strAnchorsFile;
	m_rosNodeHandle.param( "target_anchors", strAnchorsFile, string( "" ) );

	std::string strConfigurationFile;
	m_rosNodeHandle.param( "robot_configuration", strConfigurationFile, string( "" ) );

	std::string strTargetFrame;
	m_rosNodeHandle.param( "target_frame", strTargetFrame, string( "" ) );

	if( !strTargetFrame.length( ) )
	{
		strTargetFrame = "/internal/state/map/grid";
	}

	ROS_INFO_STREAM( "Stargazer: Target frame: " << strTargetFrame );

	tf::StampedTransform footprintTransform;
	footprintTransform.setRotation( tf::Quaternion::getIdentity( ) );
	footprintTransform.setOrigin( tf::Vector3( 0.39, 0.064, 0.0f ) );

	try
	{
//		ros::Time now = ros::Time( 0 );
//
//		std::string strStargazerLink( "/stargazer_camera_frame" );
//		std::string strFootprint( "/base_footprint" );
//
//		m_listener.waitForTransform( strFootprint, strStargazerLink, now, ros::Duration( 10.0 ) );
//		m_listener.lookupTransform( strFootprint, strStargazerLink, now, footprintTransform );
//
//		tf::Vector3 footprintOffset = footprintTransform.getOrigin( );
//		tf::Quaternion rotationFootprint = footprintTransform.getRotation( );
//
//		ROS_INFO_STREAM( "Stargazer base footprint offset: " << "x=" << footprintOffset.getX( ) <<
//			", y=" << footprintOffset.getY( ) << ", z=" << footprintOffset.getZ( ) << ", angle=" << tf::getYaw( rotationFootprint ) );
	}
	catch( const tf::TransformException& ex )
	{
		ROS_ERROR_STREAM( "Stargazer: Transform error: " << ex.what( ) );
	}

	tf::Transform stargazerTransform( tf::Quaternion::getIdentity( ) );

	if( LoadCalibrationTransform( strConfigurationFile, stargazerTransform ) )
	{
		m_pointTransformer.Load( strTargetFrame, stargazerTransform,
			footprintTransform, strAnchorsFile );
	}
}


bool StarGazer::LoadCalibrationTransform( const std::string& strConfigurationFile,
	tf::Transform& stargazer )
{
	// Default values
	tf::Vector3 translation( 0.0308468412, -0.0687856119, 0.0f );

	try
	{
		ROS_INFO_STREAM( "Stargazer: Calibration loading values from =>" << strConfigurationFile );

		YAML::Node document = YAML::LoadFile( strConfigurationFile );

		if( !document.IsNull( ) )
		{
			translation.setX( document["stargazer_offset"]["x"].as<double>( ) );
			translation.setX( document["stargazer_offset"]["y"].as<double>( ) );
		}
	}
	catch( const std::runtime_error& e )
	{
		ROS_ERROR_STREAM( "Stargazer: Could not load or parse yaml file for Stargazer calibration: " <<
			strConfigurationFile << ", using default values" );
	}

	stargazer.setRotation( tf::Quaternion::getIdentity( ) );
	stargazer.setOrigin( translation );

	ROS_INFO_STREAM( "Stargazer: Calibration x=" << translation.getX( ) << ", y=" << translation.getY( ) );

	return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

}// namespace srs

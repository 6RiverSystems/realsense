/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <StarGazerPointTransformer.h>
#include <srslib_framework/localization/Anchor.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <yaml-cpp/yaml.h>

using namespace boost::accumulators;

typedef accumulator_set<double, stats<tag::median> > double_acc;

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

StarGazerPointTransformer::StarGazerPointTransformer( ) :
	m_filter( 2.6, 5.0, 70 ),
	m_strAnchorFrame( "/map" ),
	m_strTargetFrame(),
	m_mapTransforms( ),
	m_stargazerTransform( ),
	m_footprintTransform( ),
	m_rotationTransform( tf::createQuaternionFromRPY( M_PI, 0.0f, 0.0 ) )
{

}

StarGazerPointTransformer::~StarGazerPointTransformer( )
{

}

double StarGazerPointTransformer::ConvertToRightHandRule( double fLeftHandAngleInDegrees ) const
{
	tf::Quaternion rightHandOrientation = tf::createQuaternionFromYaw( fLeftHandAngleInDegrees );

	tf::Quaternion leftHandOrientation = m_rotationTransform * rightHandOrientation;

	return tf::getYaw( leftHandOrientation );
}

void StarGazerPointTransformer::Load( const std::string& strTargetFrame,
	const tf::Transform& footprintTransform, const std::string& strAnchorsFile,
	const std::string& strCalibrationFile )
{
	m_strTargetFrame = strTargetFrame;

	m_footprintTransform = footprintTransform;

	tf::Vector3 origin = m_footprintTransform.getOrigin( );

	ROS_INFO_STREAM( "Stargazer footprint transform: x=" << origin.getX( ) << ", y=" << origin.getY( ) <<
		", z=" << origin.getZ( )  << ", orientation=" << tf::getYaw( m_footprintTransform.getRotation( ) ) );

	LoadAnchors( strAnchorsFile );

	LoadCalibrationTransform( strCalibrationFile );
}

tf::Quaternion StarGazerPointTransformer::ConvertAngle( double fLeftHandAngleInDegrees ) const
{
	double dfLeftHandAngleInRadians = fLeftHandAngleInDegrees * M_PI / 180.0f;

	// Transform to map coordinate system (right hand rule)
	return tf::createQuaternionFromYaw( ConvertToRightHandRule( dfLeftHandAngleInRadians ) );
}

tf::Vector3 StarGazerPointTransformer::GetStargazerOffset( double fX, double fY, double fZ ) const
{
	return tf::Vector3( fX / 100.0f, fY / 100.0f, fZ / 100.0f );
}

tf::Vector3 StarGazerPointTransformer::GetCameraOffset( const tf::Quaternion& angle ) const
{
	// Rotated around map origin
	tf::Pose poseFootprintRotated( angle );

	// Translate by optical -> camera
	tf::Pose poseCamera = poseFootprintRotated * m_stargazerTransform;

	return poseCamera.getOrigin( );
}

tf::Vector3 StarGazerPointTransformer::GetFootprintOffset( const tf::Quaternion& angle ) const
{
	// Rotated around map origin
	tf::Pose poseFootprintRotated( angle );

	// Translate by camera -> base footprint
	tf::Pose footprintPose = poseFootprintRotated * m_footprintTransform;

	return footprintPose.getOrigin( );
}

bool StarGazerPointTransformer::TransformPoint( int nTagId, double fX, double fY, double fZ, double fAngleInDegrees, tf::Pose& pose )
{
	bool bSuccess = false;

	auto iter = m_mapTransforms.find( nTagId );

	if( iter != m_mapTransforms.end( ) )
	{
		if( m_filter.unfilterStargazerData( nTagId, fX, fY, fZ ) )
		{
			// The anchor transform from the origin of the map to the anchor point
			const tf::Transform& anchorGlobal = iter->second;

			std::string strAnchorFrame = GetAnchorFrame( nTagId );

			tf::Quaternion anchorRotation = ConvertAngle( fAngleInDegrees );

			tf::Vector3 cameraOffset = GetCameraOffset( anchorRotation );

			tf::Vector3 footprintOffset = GetFootprintOffset( anchorRotation );

			tf::Vector3 stargazerOffset = GetStargazerOffset( fX, fY, fZ );

			tf::Vector3 stargazerTranslation = cameraOffset + footprintOffset + stargazerOffset;

			tf::Vector3 chuckOrigin = anchorGlobal * stargazerTranslation;

			tf::Quaternion chuckOrientation = anchorGlobal * anchorRotation;

			pose = tf::Pose( chuckOrientation, chuckOrigin);

			bSuccess = true;

			tf::Vector3 totalCameraOffset = cameraOffset + stargazerOffset;

			tf::Vector3 totalFootprintOffset = totalCameraOffset + footprintOffset;

//			std::ostringstream stream;
//
//			stream << "Stargazer Data: " << std::fixed <<  endl <<
//				tf::getYaw( anchorRotation ) * 180.0f / M_PI << ", " <<
//				stargazerOffset.getX( ) << ", " << stargazerOffset.getY( ) << ", " << stargazerOffset.getZ( ) << ", " <<
//				totalCameraOffset.getX( ) << ", " << totalCameraOffset.getY( ) << ", " << totalCameraOffset.getZ( ) << ", " <<
//				totalFootprintOffset.getX( ) << ", " << totalFootprintOffset.getY( ) << ", " << totalFootprintOffset.getZ( ) << ", " <<
//				chuckOrigin.getX( ) << ", " << chuckOrigin.getY( ) << ", " << chuckOrigin.getZ( ) << ", " <<
//				tf::getYaw( chuckOrientation ) * 180.0f / M_PI << ", " << std::endl;
//
//			std::string strData =  stream.str( );
//
//			ROS_DEBUG_NAMED( "StarGazerPointTransformer", "%s", strData.c_str( ) );

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

			accCameraX( totalCameraOffset.getX( ) );
			accCameraY( totalCameraOffset.getY( ) );
			accCameraZ( totalCameraOffset.getZ( ) );

			accFootprintX( totalFootprintOffset.getX( ) );
			accFootprintY( totalFootprintOffset.getY( ) );
			accFootprintZ( totalFootprintOffset.getZ( ) );

			accChuckX( chuckOrigin.getX( ) );
			accChuckY( chuckOrigin.getY( ) );
			accChuckZ( chuckOrigin.getZ( ) );
			accChuckRotation( tf::getYaw( chuckOrientation ) * 180.0f / M_PI );

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

				ROS_ERROR_NAMED( "StarGazerPointTransformer", "%s", strData.c_str( ) );

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
			ROS_DEBUG_NAMED( "StarGazerPointTransformer", "Rejected anchor position: %04i (%2.2f, %2.2f, %2.2f) %2.2f degrees\n",
				nTagId, fX, fY, fZ, fAngleInDegrees );
		}
	}
	else
	{
		m_filter.reportDeadZone( );

		ROS_DEBUG_NAMED( "StarGazerPointTransformer", "Invalid or Unknown Tag: %04i (%2.2f, %2.2f, %2.2f) %2.2f degrees\n",
			nTagId, fX, fY, fZ, fAngleInDegrees );
	}

	return bSuccess;
}

std::string StarGazerPointTransformer::GetTargetFrame( ) const
{
	return m_strTargetFrame;
}

std::string StarGazerPointTransformer::GetAnchorFrame( int tagID ) const
{
	std::ostringstream stringStream;
	stringStream << "/aps_anchor_";
	stringStream << tagID;

	return std::move( stringStream.str( ) );
}

void StarGazerPointTransformer::LoadAnchors( const std::string& strAnchorsFile )
{
	try
	{
		YAML::Node document = YAML::LoadFile( strAnchorsFile );

		if( !document.IsNull( ) )
		{
			vector<Anchor> anchors = document["anchors"].as<vector<Anchor>>( );

			for( auto anchor : anchors )
			{
				try
				{
					int32_t anchorId = boost::lexical_cast < int32_t > (anchor.id);

					tf::Vector3 origin( anchor.x, anchor.y, -anchor.z );
					tf::Quaternion orientation = tf::createQuaternionFromYaw( anchor.orientation );

					tf::Transform transform( orientation, origin );

					ROS_INFO_STREAM( "Stargazer anchor: x=" << origin.getX( ) << ", y=" << origin.getY( ) <<
						", z=" << origin.getZ( )  << ", orientation=" << tf::getYaw( orientation ) );

					m_mapTransforms[anchorId] = transform;
				}
				catch( const boost::bad_lexical_cast& e )
				{
					ROS_ERROR_STREAM( "Could not convert anchor id to int: " << anchor.id << " " << e.what( ) );
				}
			}
		}
		else
		{
			ROS_ERROR_STREAM( "Configuration file not found: " << strAnchorsFile );
		}
	}
	catch( const std::runtime_error& e )
	{
		ROS_ERROR_STREAM( "Could not parse yaml file for anchors: " << strAnchorsFile << " " << e.what( ) );
	}
}

void StarGazerPointTransformer::LoadCalibrationTransform( const std::string& strConfigurationFile )
{
	try
	{
		YAML::Node document = YAML::LoadFile( strConfigurationFile );

		if( !document.IsNull( ) )
		{
			tf::Vector3 translation(
				document["stargazer_offset"]["x"].as<double>( ),
				document["stargazer_offset"]["y"].as<double>( ),
				0.0f
			);

			m_stargazerTransform.setRotation( tf::Quaternion::getIdentity( ) );
			m_stargazerTransform.setOrigin( translation );

			ROS_INFO_STREAM( "Stargazer calibration: x=" << translation.getX( ) << ", y=" << translation.getY( ) );
		}
		else
		{
			ROS_ERROR_STREAM( "Stargazer calibration file not found: " << strConfigurationFile );
		}
	}
	catch( const std::runtime_error& e )
	{
		ROS_ERROR_STREAM( "Could not parse yaml file for Stargazer calibration: " << strConfigurationFile << " " << e.what( ) );
	}
}

}

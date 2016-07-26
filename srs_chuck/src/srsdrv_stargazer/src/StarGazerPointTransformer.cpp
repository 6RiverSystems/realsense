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
#include <boost/accumulators/statistics/variance.hpp>
#include <yaml-cpp/yaml.h>

using namespace boost::accumulators;

typedef accumulator_set<double, stats<tag::median, tag::lazy_variance> > double_acc;

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

// From calibrated Natasha
auto constexpr DEFAULT_OFFSET_X = 0.0371973746;
auto constexpr DEFAULT_OFFSET_Y = -0.0697303969746;

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

StarGazerPointTransformer::StarGazerPointTransformer( ) :
	m_filter( 2.6, 5.0, 70 ),
	m_strAnchorFrame( "/internal/state/map/grid" ),
	m_strTargetFrame(),
	m_mapTransforms( ),
	m_stargazerTransform( tf::Quaternion::getIdentity( ), tf::Vector3( DEFAULT_OFFSET_X, DEFAULT_OFFSET_Y, 0.0f ) ),
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

bool StarGazerPointTransformer::Load( const std::string& strTargetFrame,
	const tf::Transform& stargazerTransform, const tf::Transform& footprintTransform,
	const std::string& strAnchorsFile )
{
	m_strTargetFrame = strTargetFrame;

	m_footprintTransform = footprintTransform;

	m_stargazerTransform = stargazerTransform;

	tf::Vector3 origin = m_footprintTransform.getOrigin( );

	ROS_DEBUG_STREAM( "Stargazer: Footprint transform: x=" << origin.getX( ) << ", y=" << origin.getY( ) <<
		", z=" << origin.getZ( )  << ", orientation=" << tf::getYaw( m_footprintTransform.getRotation( ) ) );

	return LoadAnchors( strAnchorsFile );
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

bool StarGazerPointTransformer::TransformPoint( int nTagId, double fX, double fY, double fZ,
	double fAngleInDegrees, tf::Pose& pose, bool bFilter )
{
	bool bSuccess = false;

	auto iter = m_mapTransforms.find( nTagId );

	if( iter != m_mapTransforms.end( ) )
	{
		if( !bFilter || m_filter.unfilterStargazerData( nTagId, fX, fY, fZ ) )
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

			double dfAnchorGlobal = tf::getYaw(anchorGlobal.getRotation( ) );
			double dfAnchorRotation = tf::getYaw( anchorRotation );
			double dfMapRotation = dfAnchorGlobal + dfAnchorRotation;

			tf::Quaternion chuckOrientation( tf::createQuaternionFromYaw( dfMapRotation ) );
			pose = tf::Pose( chuckOrientation, chuckOrigin);

			bSuccess = true;

			tf::Vector3 totalCameraOffset = cameraOffset + stargazerOffset;

			tf::Vector3 totalFootprintOffset = totalCameraOffset + footprintOffset;

			std::ostringstream stream;

			double dfMapRotationDegrees = dfMapRotation * 180.0f / M_PI;

			if( dfMapRotationDegrees < 0 )
			{
				dfMapRotationDegrees += 360.0f;
			}

			stream << "Stargazer: OM Data" << nTagId << std::fixed << ", " <<
				tf::getYaw( anchorRotation ) * 180.0f / M_PI << ", " <<
				stargazerOffset.getX( ) << ", " << stargazerOffset.getY( ) << ", " << stargazerOffset.getZ( ) << ", " <<
				totalCameraOffset.getX( ) << ", " << totalCameraOffset.getY( ) << ", " << totalCameraOffset.getZ( ) << ", " <<
				totalFootprintOffset.getX( ) << ", " << totalFootprintOffset.getY( ) << ", " << totalFootprintOffset.getZ( ) << ", " <<
				chuckOrigin.getX( ) << ", " << chuckOrigin.getY( ) << ", " << chuckOrigin.getZ( ) << ", " <<
				dfMapRotationDegrees << ", " << std::endl;

			std::string strData =  stream.str( );

			ROS_DEBUG_STREAM_NAMED( "transform", strData );

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
			accChuckRotation( dfMapRotationDegrees );

			static int sCount = 0;
			if( sCount++ == 50 )
			{
				std::ostringstream stream;

				stream << "Stargazer: OM Calibration Data => " << nTagId << std::fixed << ", " <<
					tf::getYaw( anchorRotation ) * 180.0f / M_PI << ", " <<
					median( accLocalX ) << ", " << median( accLocalY ) << ", " << median( accLocalZ ) << ", " <<
					median( accCameraX ) << ", " << median( accCameraY ) << ", " << median( accCameraZ ) << ", " <<
					median( accFootprintX ) << ", " << median( accFootprintY ) << ", " << median( accFootprintZ ) << ", " <<
					median( accChuckX ) << ", " << median( accChuckY ) << ", " << median( accChuckZ ) << ", " <<
					median( accChuckRotation ) << ", " <<
					lazy_variance( accChuckX ) << ", " << lazy_variance( accChuckY ) << ", " << lazy_variance( accChuckZ ) << ", " <<
					lazy_variance( accChuckRotation ) <<
					std::endl;

				std::string strData =  stream.str( );

				ROS_INFO_STREAM_NAMED( "calibrate", strData );

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
			ROS_DEBUG_NAMED( "transform", "Stargazer: Rejected anchor position: %04i (%2.2f, %2.2f, %2.2f) %2.2f degrees\n",
				nTagId, fX, fY, fZ, fAngleInDegrees );
		}
	}
	else
	{
		m_filter.reportDeadZone( );

		ROS_DEBUG_NAMED( "calibrate", "Stargazer: Invalid or Unknown Tag: %04i (%2.2f, %2.2f, %2.2f) %2.2f degrees\n",
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

bool StarGazerPointTransformer::LoadAnchors( const std::string& strAnchorsFile )
{
	bool bSuccess = true;

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
					tf::Quaternion orientation = tf::createQuaternionFromYaw( anchor.orientation * M_PI / 180.0f );

					tf::Transform transform( orientation, origin );

					ROS_INFO_STREAM( "Stargazer: anchor: x=" << origin.getX( ) << ", y=" << origin.getY( ) <<
						", z=" << origin.getZ( )  << ", orientation=" << tf::getYaw( orientation ) * 180.0f / M_PI );

					m_mapTransforms[anchorId] = transform;
				}
				catch( const boost::bad_lexical_cast& e )
				{
					ROS_ERROR_STREAM( "Stargazer: Could not convert anchor id to int: " << anchor.id << " " << e.what( ) );

					bSuccess = false;
				}
			}
		}
		else
		{
			ROS_ERROR_STREAM( "Stargazer: Configuration file not found: " << strAnchorsFile );

			bSuccess = false;
		}
	}
	catch( const std::runtime_error& e )
	{
		ROS_ERROR_STREAM( "Stargazer: Could not parse yaml file for anchors: " << strAnchorsFile << " " << e.what( ) );

		bSuccess = false;
	}

	return bSuccess;
}

}

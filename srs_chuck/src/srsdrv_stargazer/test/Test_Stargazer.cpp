/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <StarGazerPointTransformer.h>

#include "gtest/gtest.h"
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>
#include <sys/stat.h>
#include <math.h>
#include <boost/filesystem.hpp>

#include <srslib_framework/math/AngleMath.hpp>
#include <srslib_framework/chuck/ChuckTopics.hpp>

namespace srs {

std::string g_strDataFile;

const auto FOOTPRINT_OFFSET_X = 7.0f;
const auto FOOTPRINT_OFFSET_Y = 13.0f;

struct Point
{
	double x; // In meters
	double y; // In meters
	double z; // In meters
	double angle; // In degrees
};

class StargazerTest : public ::testing::Test
{
public:

	std::string						m_strAnchorFile;

	StarGazerPointTransformer  		m_pointTransformer;

	tf::Transform					m_footprintTransform;

	std::vector<Point>				m_vecTestPoints;

	std::vector<Point>				m_vecStargazerOffsets;

public:

	StargazerTest( ) :
		m_strAnchorFile( g_strDataFile + "/anchors.yaml"),
		m_footprintTransform( tf::Quaternion::getIdentity( ),
			tf::Vector3( FOOTPRINT_OFFSET_X, FOOTPRINT_OFFSET_Y, 0.0 ) )
	{
		boost::filesystem::path currentPath( boost::filesystem::current_path( ) );

		m_strAnchorFile = currentPath.c_str( );
		m_strAnchorFile += "/anchors.yaml";

		ROS_DEBUG_NAMED( "transform", "Stargazer: Anchor file: %s", m_strAnchorFile.c_str( ) );

		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, 0.0f } );
		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, 90.0f } );
		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, 180.0f } );
		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, -90.0f } );

		m_vecStargazerOffsets.push_back( { 3.0f, 5.0f, 0.0f, 0.0f } );
		m_vecStargazerOffsets.push_back( { -3.0f, 5.0f, 0.0f, 90.0f } );
		m_vecStargazerOffsets.push_back( { 3.0f, -5.0f, 0.0f, 180.0f } );
		m_vecStargazerOffsets.push_back( { -3.0f, -5.0f, 0.0f, -90.0f } );
	}

	void SetUp( )
	{

	}

	void TearDown( )
	{

	}

	~StargazerTest( )
	{

	}

	tf::Pose GetCalculatedPose( const tf::Transform& anchorTransform, const tf::Transform& stargazerTransform,
		const Point& rawPoint )
	{
		// Conversion works the same in both directions
		double markerToMapRotation = tf::getYaw( anchorTransform.getRotation( ) );

		double cameraToMarkerRotation = AngleMath::normalizeDeg2Rad( rawPoint.angle );
		double chuckToCameraRotation = tf::getYaw( stargazerTransform.getRotation( ) );

		double chuckToMarkerRotation = chuckToCameraRotation + cameraToMarkerRotation;

		double combinedAngle = AngleMath::normalizeRad<double>( markerToMapRotation + chuckToMarkerRotation );

		tf::Vector3 anchorOrigin = anchorTransform.getOrigin( );

		tf::Vector3 stargazerOffset = stargazerTransform.getOrigin( );

		tf::Vector3 footprintOffset = m_footprintTransform.getOrigin( );

		tf::Vector3 combinedOffset = stargazerOffset + footprintOffset;

		// Calculate: stargazer offset => stargazer => base_footprint
		tf::Vector3 rotatedOffset(
			((combinedOffset.getX( ) * cos( chuckToMarkerRotation )) - (combinedOffset.getY( ) * sin( chuckToMarkerRotation ))),
			((combinedOffset.getX( ) * sin( chuckToMarkerRotation )) + (combinedOffset.getY( ) * cos( chuckToMarkerRotation ))),
			0.0f );

		// Convert cm to m
		Point point = { rawPoint.x / 100.0f, rawPoint.y / 100.0f, rawPoint.z / 100.0f};

		// Translate: stargazer => base_footprint
		tf::Vector3 offsetFromStargazer( point.x, point.y, point.z );
		offsetFromStargazer += rotatedOffset;

		// Rotate: anchor => map
		tf::Vector3 globalPoint(
			((offsetFromStargazer.getX( ) * cos( markerToMapRotation )) - (offsetFromStargazer.getY( ) * sin( markerToMapRotation ))),
			((offsetFromStargazer.getX( ) * sin( markerToMapRotation )) + (offsetFromStargazer.getY( ) * cos( markerToMapRotation ))),
			offsetFromStargazer.getZ( ) );

		// Translate: anchor => map
		globalPoint += anchorOrigin;

		return tf::Pose( tf::createQuaternionFromYaw( combinedAngle ), globalPoint );
	}

	double GetLeftHandAngle( double dfRightHandDegrees )
	{
		double dfLeftHandAngleInRadians = m_pointTransformer.ConvertToRightHandRule( AngleMath::deg2Rad<double>( dfRightHandDegrees ) );

		return AngleMath::normalizeRad2Deg( dfLeftHandAngleInRadians );
	}

};

TEST_F( StargazerTest, TestAverageAngle )
{
	tf::Transform stargazerTransform( tf::Quaternion::getIdentity( ), tf::Vector3( 0.0, 0.0, 0.0) );

	m_pointTransformer.Load(ChuckTopics::internal::MAP_ROS_OCCUPANCY, stargazerTransform,
		m_footprintTransform, m_strAnchorFile );

	double fourtyFive = 45.0;
	double negativeFourtyFive = -45.0;

	for( int i = 0; i < 25; i++ )
	{
		tf::Pose pose( tf::createIdentityQuaternion( ) );

		m_pointTransformer.TransformPoint( 1, 0.0, 0.0, 0.0, fourtyFive, pose, false );
	}

	for( int i = 0; i < 25; i++ )
	{
		tf::Pose pose( tf::createIdentityQuaternion( ) );

		m_pointTransformer.TransformPoint( 1, 0.0, 0.0, 0.0, negativeFourtyFive, pose, false );
	}
}

TEST_F( StargazerTest, TestCameraAngleCalibration )
{
	tf::Transform stargazerTransform( tf::Quaternion::getIdentity( ), tf::Vector3( 0.0, 0.0, 0.0) );

	m_pointTransformer.Load(ChuckTopics::internal::MAP_ROS_OCCUPANCY, stargazerTransform,
		m_footprintTransform, m_strAnchorFile );

	tf::Pose pose( tf::createIdentityQuaternion( ) );
	m_pointTransformer.TransformPoint( 1, 0.0, 0.0, 0.0, 0.0, pose, false );
}

TEST_F( StargazerTest, TestRightHandRuleConverstion )
{
	double df0Degrees = AngleMath::normalizeRad2Deg<double>( m_pointTransformer.ConvertToRightHandRule( AngleMath::deg2Rad<double>( 0.0) ) );
	double df1Degrees = AngleMath::normalizeRad2Deg<double>( m_pointTransformer.ConvertToRightHandRule( AngleMath::deg2Rad<double>( 1.0) ) );
	double df45Degrees = AngleMath::normalizeRad2Deg<double>( m_pointTransformer.ConvertToRightHandRule( AngleMath::deg2Rad<double>( 45.0) ) );
	double df90Degrees = AngleMath::normalizeRad2Deg<double>( m_pointTransformer.ConvertToRightHandRule( AngleMath::deg2Rad<double>( 90.0) ) );
	double df135Degrees = AngleMath::normalizeRad2Deg<double>( m_pointTransformer.ConvertToRightHandRule( AngleMath::deg2Rad<double>( 135.0) ) );
	double df180Degrees = AngleMath::normalizeRad2Deg<double>( m_pointTransformer.ConvertToRightHandRule( AngleMath::deg2Rad<double>( 180.0) ) );
	double df225Degrees = AngleMath::normalizeRad2Deg<double>( m_pointTransformer.ConvertToRightHandRule( AngleMath::deg2Rad<double>( 225.0) ) );
	double df270Degrees = AngleMath::normalizeRad2Deg<double>( m_pointTransformer.ConvertToRightHandRule( AngleMath::deg2Rad<double>( 270.0) ) );
	double df315Degrees = AngleMath::normalizeRad2Deg<double>( m_pointTransformer.ConvertToRightHandRule( AngleMath::deg2Rad<double>( 315.0) ) );

	EXPECT_NEAR( df0Degrees, 0.0f, 0.001 );
	EXPECT_NEAR( df1Degrees, 359.0f, 0.001 );
	EXPECT_NEAR( df45Degrees, 315.0f, 0.001 );
	EXPECT_NEAR( df90Degrees, 270.0f, 0.001 );
	EXPECT_NEAR( df135Degrees, 225.0f, 0.001 );
	EXPECT_NEAR( df180Degrees, 180.0f, 0.001 );
	EXPECT_NEAR( df225Degrees, 135.0f, 0.001 );
	EXPECT_NEAR( df270Degrees, 90.0f, 0.001 );
	EXPECT_NEAR( df315Degrees, 45.0f, 0.001 );
}

TEST_F( StargazerTest, TestTransforms )
{
	for( auto stargazerOffset : m_vecStargazerOffsets )
	{
		tf::Transform stargazerTransform( tf::createQuaternionFromYaw(
			AngleMath::normalizeDeg2Rad( stargazerOffset.angle ) ),
			tf::Vector3( stargazerOffset.x, stargazerOffset.y, stargazerOffset.z) );

		m_pointTransformer.Load(ChuckTopics::internal::MAP_ROS_OCCUPANCY, stargazerTransform,
			m_footprintTransform, m_strAnchorFile );

		std::map<int, tf::Transform> mapAnchorTransforms = m_pointTransformer.GetAnchorTransforms( );
		for( auto anchorPair : mapAnchorTransforms )
		{
			uint32_t anchorId = anchorPair.first;

			tf::Transform anchorTransform = mapAnchorTransforms[anchorId];

			double dfAngle = AngleMath::normalizeRad2Deg( tf::getYaw( anchorTransform.getRotation( ) ) );

			tf::Vector3 origin = anchorTransform.getOrigin( );

			ROS_DEBUG_NAMED( "transform", "Stargazer: Testing anchor: id: %d, x: %2.5f y: %2.5f z: %2.5f angle: %2.5f",
				anchorId, origin.getX( ), origin.getY( ), origin.getZ( ), dfAngle );

			for( auto point : m_vecTestPoints )
			{
				auto testPoint = point;

				ROS_DEBUG_NAMED( "transform", "\tTesting point: x: %2.5f y: %2.5f z: %2.5f angle: %2.5f",
					testPoint.x, testPoint.y, testPoint.z, testPoint.angle );

				// Convert to left hand degrees from right hand degrees
				double dfAngleLeftDegrees = GetLeftHandAngle( testPoint.angle );

				tf::Quaternion rotation = m_pointTransformer.ConvertAngle( dfAngleLeftDegrees );

				tf::Quaternion combinedRotation = tf::createQuaternionFromYaw( tf::getYaw( rotation ) + tf::getYaw( stargazerTransform.getRotation( ) ) );

				tf::Vector3 cameraOffset = m_pointTransformer.GetCameraOffset( combinedRotation );

				tf::Vector3 footprintOffset = m_pointTransformer.GetFootprintOffset( combinedRotation );

				tf::Vector3 stargazerOffset = m_pointTransformer.GetStargazerOffset( testPoint.x*100.0f, testPoint.y*100.0f, testPoint.z*100.0f );

				tf::Vector3 totalCameraOffset = cameraOffset + stargazerOffset;

				tf::Vector3 totalFootprintOffset = totalCameraOffset + footprintOffset;

				ROS_DEBUG_NAMED( "transform", "\t\tstargazer (%2.5f): %2.5f, %2.5f, %2.5f",
					AngleMath::normalizeRad2Deg( tf::getYaw( rotation ) ), stargazerOffset.getX( ), stargazerOffset.getY( ), stargazerOffset.getZ( ) );

				ROS_DEBUG_NAMED( "transform", "\t\tcamera (%2.5f): %2.5f, %2.5f, %2.5f",
					AngleMath::normalizeRad2Deg( tf::getYaw( rotation ) ), totalCameraOffset.getX( ), totalCameraOffset.getY( ), totalCameraOffset.getZ( ) );

				ROS_DEBUG_NAMED( "transform", "\t\tfootprint (%2.5f): %2.5f, %2.5f, %2.5f",
					AngleMath::normalizeRad2Deg( tf::getYaw( rotation ) ), totalFootprintOffset.getX( ), totalFootprintOffset.getY( ), totalFootprintOffset.getZ( ) );

				tf::Pose pose( tf::createIdentityQuaternion( ) );

				// Convert to cm
				testPoint.x *= 100.0f;
				testPoint.y *= 100.0f;
				testPoint.z *= 100.0f;

				if( m_pointTransformer.TransformPoint( anchorId, testPoint.x, testPoint.y,
					testPoint.z, dfAngleLeftDegrees, pose, false ) )
				{
					tf::Pose calculatedPose = GetCalculatedPose( anchorTransform, stargazerTransform, testPoint );

					tf::Vector3 origin = pose.getOrigin( );
					tf::Quaternion rotation = pose.getRotation( );

					ROS_DEBUG_NAMED( "transform", "\t\tmap (%2.5f): %2.5f, %2.5f, %2.5f",
						AngleMath::normalizeRad2Deg( tf::getYaw( rotation ) ), origin.getX( ), origin.getY( ), origin.getZ( ) );

					tf::Vector3 calculatedOrigin = calculatedPose.getOrigin( );
					tf::Quaternion calculatedRotation = calculatedPose.getRotation( );

					ROS_DEBUG_NAMED( "transform", "\t\tmap (correct) (%2.5f): %2.5f, %2.5f, %2.5f",
						AngleMath::normalizeRad2Deg( tf::getYaw( calculatedRotation ) ), calculatedOrigin.getX( ),
						calculatedOrigin.getY( ), calculatedOrigin.getZ( ) );

					EXPECT_NEAR( origin.getX( ), calculatedOrigin.getX( ), 0.0000001 );
					EXPECT_NEAR( origin.getY( ), calculatedOrigin.getY( ), 0.0000001 );
					EXPECT_NEAR( origin.getZ( ), calculatedOrigin.getZ( ), 0.0000001 );
					EXPECT_NEAR( tf::getYaw( rotation ), tf::getYaw( calculatedRotation ), 0.0000001 );
				}
			}
		}
	}
}

}  // namespace

int main(int argc, char **argv)
{
	::testing::InitGoogleTest( &argc, argv );

	return RUN_ALL_TESTS( );
}

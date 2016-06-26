/*
 * Test_Stargazer.cpp
 *
 *  Created on: Apr 4, 2016
 *      Author: dan
 */

#include <StarGazerPointTransformer.h>
#include "gtest/gtest.h"
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>
#include <sys/stat.h>
#include <math.h>

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
		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, 0.0f } );
		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, 90.0f } );
		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, 180.0f } );
		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, -90.0f } );

		m_vecStargazerOffsets.push_back( { 3.0f, 5.0f } );
		m_vecStargazerOffsets.push_back( { -3.0f, 5.0f } );
		m_vecStargazerOffsets.push_back( { 3.0f, -5.0f } );
		m_vecStargazerOffsets.push_back( { -3.0f, -5.0f } );
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
		double stargazerRotation = rawPoint.angle * M_PI / 180.0;
		double anchorRotation = tf::getYaw( anchorTransform.getRotation( ) );

		double combinedAngle = anchorRotation + stargazerRotation;

		tf::Vector3 anchorOrigin = anchorTransform.getOrigin( );

		tf::Vector3 stargazerOffset = stargazerTransform.getOrigin( );

		tf::Vector3 footprintOffset = m_footprintTransform.getOrigin( );

		tf::Vector3 combinedOffset = stargazerOffset + footprintOffset;

		// Calculate: stargazer offset => stargazer => base_footprint
		tf::Vector3 rotatedOffset(
			((combinedOffset.getX( ) * cos( stargazerRotation )) - (combinedOffset.getY( ) * sin( stargazerRotation ))),
			((combinedOffset.getX( ) * sin( stargazerRotation )) + (combinedOffset.getY( ) * cos( stargazerRotation ))),
			0.0f );

		// Convert cm to m
		Point point = { rawPoint.x / 100.0f, rawPoint.y / 100.0f, rawPoint.z / 100.0f};

		// Translate: stargazer => base_footprint
		tf::Vector3 offsetFromStargazer( point.x, point.y, point.z );
		offsetFromStargazer += rotatedOffset;

		// Rotate: anchor => map
		tf::Vector3 globalPoint(
			((offsetFromStargazer.getX( ) * cos( anchorRotation )) - (offsetFromStargazer.getY( ) * sin( anchorRotation ))),
			((offsetFromStargazer.getX( ) * sin( anchorRotation )) + (offsetFromStargazer.getY( ) * cos( anchorRotation ))),
			offsetFromStargazer.getZ( ) );

		// Translate: anchor => map
		globalPoint += anchorOrigin;

		return tf::Pose( tf::createQuaternionFromYaw( combinedAngle ), globalPoint );
	}

	double GetLeftHandAngle( double dfRightHandDegrees )
	{
		double dfLeftHandAngleInRadians = m_pointTransformer.ConvertToRightHandRule( dfRightHandDegrees * M_PI / 180 );

		return dfLeftHandAngleInRadians * 180 / M_PI;
	}

};

TEST_F( StargazerTest, TestTransforms )
{
	uint32_t testIndex = 0;

	char pszConfigTest[255] = { '\0' };

	for( auto stargazerOffset : m_vecStargazerOffsets )
	{
		tf::Transform stargazerTransform( tf::Quaternion::getIdentity( ), tf::Vector3( stargazerOffset.x,
			stargazerOffset.y, stargazerOffset.z) );

		m_pointTransformer.Load( "/internal/state/map/grid", stargazerTransform,
			m_footprintTransform, m_strAnchorFile );

		std::map<int, tf::Transform> mapAnchorTransforms = m_pointTransformer.GetAnchorTransforms( );

		for( auto point : m_vecTestPoints )
		{
			for( auto anchorPair : mapAnchorTransforms )
			{
				uint32_t anchorId = anchorPair.first;

				auto testPoint = point;

				tf::Transform anchorTransform = mapAnchorTransforms[anchorId];

				tf::Vector3 origin = anchorTransform.getOrigin( );

				double dfAngle = tf::getYaw( anchorTransform.getRotation( ) ) * 180.0 / M_PI;

				ROS_INFO_NAMED( "transform", "Testing config: %s", pszConfigTest );
				ROS_INFO_NAMED( "transform", "Testing anchor: id: %d, x: %2.5f y: %2.5f z: %2.5f angle: %2.5f",
					anchorId, origin.getX( ), origin.getY( ), origin.getZ( ), dfAngle );
				ROS_INFO_NAMED( "transform", "Testing point: x: %2.5f y: %2.5f z: %2.5f angle: %2.5f",
					testPoint.x, testPoint.y, testPoint.z, testPoint.angle );

				// Convert to left hand degrees from right hand degrees
				double dfAngleLeftDegrees = GetLeftHandAngle( testPoint.angle );

				tf::Quaternion rotation = m_pointTransformer.ConvertAngle( dfAngleLeftDegrees );

				double dfAngleRightRadians = tf::getYaw( rotation );

				double dfAngleRightDegrees = dfAngleRightRadians * 180.0f / M_PI;

				tf::Vector3 cameraOffset = m_pointTransformer.GetCameraOffset( rotation );

				tf::Vector3 footprintOffset = m_pointTransformer.GetFootprintOffset( rotation );

				tf::Vector3 stargazerOffset = m_pointTransformer.GetStargazerOffset( testPoint.x*100.0f, testPoint.y*100.0f, testPoint.z*100.0f );

				tf::Vector3 totalCameraOffset = cameraOffset + stargazerOffset;

				tf::Vector3 totalFootprintOffset = totalCameraOffset + footprintOffset;

				ROS_INFO_NAMED( "StarGazerPointTransformer", "stargazer (%2.5f): %2.5f, %2.5f, %2.5f",
					tf::getYaw( rotation ) * 180.0f / M_PI, stargazerOffset.getX( ), stargazerOffset.getY( ), stargazerOffset.getZ( ) );

				ROS_INFO_NAMED( "StarGazerPointTransformer", "camera (%2.5f): %2.5f, %2.5f, %2.5f",
					tf::getYaw( rotation ) * 180.0f / M_PI, totalCameraOffset.getX( ), totalCameraOffset.getY( ), totalCameraOffset.getZ( ) );

				ROS_INFO_NAMED( "StarGazerPointTransformer", "footprint (%2.5f): %2.5f, %2.5f, %2.5f",
					tf::getYaw( rotation ) * 180.0f / M_PI, totalFootprintOffset.getX( ), totalFootprintOffset.getY( ), totalFootprintOffset.getZ( ) );

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

					ROS_INFO_NAMED( "StarGazerPointTransformer", "map (%2.5f): %2.5f, %2.5f, %2.5f",
						tf::getYaw( rotation ) * 180.0f / M_PI, origin.getX( ), origin.getY( ), origin.getZ( ) );

					tf::Vector3 calculatedOrigin = calculatedPose.getOrigin( );
					tf::Quaternion calculatedRotation = calculatedPose.getRotation( );


					ROS_INFO_NAMED( "StarGazerPointTransformer", "map (correct) (%2.5f): %2.5f, %2.5f, %2.5f",
						tf::getYaw( calculatedRotation ) * 180.0f / M_PI, calculatedOrigin.getX( ),
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

	srs::g_strDataFile = "/home/dan/ros/srs_chuck/src/srsdrv_stargazer/data";

	printf("Test File: %s", srs::g_strDataFile.c_str( ) );

	return RUN_ALL_TESTS( );
}

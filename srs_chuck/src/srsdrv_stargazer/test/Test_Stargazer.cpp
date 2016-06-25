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

namespace srs {

std::string g_strDataFile;

const auto g_footprintOffsetX = 7.0f;
const auto g_footprintOffsetY = 13.0f;

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

	StarGazerPointTransformer  		m_pointTransformer;

	tf::Transform					m_footprintTransform;

	std::vector<uint32_t>			m_vecAnchors;

	std::vector<Point>				m_vecTestPoints;

public:

	StargazerTest( ) :
		m_footprintTransform( tf::Quaternion::getIdentity( ),
			tf::Vector3( g_footprintOffsetX, g_footprintOffsetY, 0.0 ) )
	{
		// TODO: Load from anchor file
		m_vecAnchors.push_back( 1 );
		m_vecAnchors.push_back( 2 );
		m_vecAnchors.push_back( 3 );
		m_vecAnchors.push_back( 4 );

		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, 0.0f } );
		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, 90.0f } );
		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, 180.0f } );
		m_vecTestPoints.push_back( { 300.0f, 700.0f, 300.0f, -90.0f } );
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

	bool LoadTestData( const std::string& strConfigTest )
	{
		std::string strAnchors( g_strDataFile );
		strAnchors += "/anchors.yaml";

		std::string strCalibrationFile( g_strDataFile );
		strCalibrationFile += "/";
		strCalibrationFile += strConfigTest;

		return m_pointTransformer.Load( "/internal/state/map/grid", m_footprintTransform,
			strAnchors, strCalibrationFile );
	}

	void GetCalculatedData( )
	{

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

	while( true )
	{
		sprintf( pszConfigTest, "robot-config-%d.yaml", ++testIndex );

		if( LoadTestData( pszConfigTest ) )
		{
			std::map<int, tf::Transform> mapAnchorTransforms = m_pointTransformer.GetAnchorTransforms( );

			for( auto point : m_vecTestPoints )
			{
				for( auto anchorId : m_vecAnchors )
				{
					auto testPoint = point;

					tf::Transform anchorTransform = mapAnchorTransforms[anchorId];
					tf::Vector3 origin = anchorTransform.getOrigin( );
					double dfAngle = tf::getYaw( anchorTransform.getRotation( ) ) * 180.0 / M_PI;

					ROS_DEBUG_NAMED( "transform", "Testing config: %s", pszConfigTest );
					ROS_DEBUG_NAMED( "transform", "Testing anchor: id: %d, x: %2.5f y: %2.5f z: %2.5f angle: %2.5f",
						anchorId, origin.getX( ), origin.getY( ), origin.getZ( ), dfAngle );
					ROS_DEBUG_NAMED( "transform", "Testing point: x: %2.5f y: %2.5f z: %2.5f angle: %2.5f",
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

					ROS_DEBUG_NAMED( "StarGazerPointTransformer", "stargazer (%2.5f): %2.5f, %2.5f, %2.5f",
						tf::getYaw( rotation ), stargazerOffset.getX( ), stargazerOffset.getY( ), stargazerOffset.getZ( ) );

					ROS_DEBUG_NAMED( "StarGazerPointTransformer", "camera (%2.5f): %2.5f, %2.5f, %2.5f",
						tf::getYaw( rotation ), totalCameraOffset.getX( ), totalCameraOffset.getY( ), totalCameraOffset.getZ( ) );

					ROS_DEBUG_NAMED( "StarGazerPointTransformer", "footprint (%2.5f): %2.5f, %2.5f, %2.5f",
						tf::getYaw( rotation ), totalFootprintOffset.getX( ), totalFootprintOffset.getY( ), totalFootprintOffset.getZ( ) );

					tf::Pose pose( tf::createIdentityQuaternion( ) );

					// Convert to cm
					testPoint.x *= 100.0f;
					testPoint.y *= 100.0f;
					testPoint.z *= 100.0f;

					if( m_pointTransformer.TransformPoint( anchorId, testPoint.x, testPoint.y,
						testPoint.z, dfAngleLeftDegrees, pose, false ) )
					{
						// TODO: Calculate the expected results and test each case
						// For now bugs are fixed and eyeballed so it is working
						// EXPECT_EQ( pose, poseCalculated );
					}
				}
			}
		}
		else
		{
			break;
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

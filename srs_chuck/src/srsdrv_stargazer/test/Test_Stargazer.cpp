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

struct Pose
{
	Pose() :
		x(0),
		y(0),
		z(0),
		orientation()
	{}

	Pose(unsigned int x, unsigned int y, unsigned int z, double orientation) :
		x(0),
		y(0),
		z(0),
		orientation()
	{}

	bool decode( const YAML::Node& node )
	{
		x = node["x"].as<double>( );
		y = node["y"].as<double>( );
		z = node["z"].as<double>( );
		orientation = node["orientation"].as<double>( );

		return true;
	}

    double x;
    double y;
    double z;

    double orientation;
};

struct StargazerMeasurement
{
	StargazerMeasurement() :
		rawPose(),
		cameraPose(),
		footprintPose(),
		mapPose()
	{}

	StargazerMeasurement(Pose rawPose, Pose cameraPose, Pose footprintPose, Pose mapPose) :
		rawPose(),
		cameraPose(),
		footprintPose(),
		mapPose()
	{}

	bool decode( const YAML::Node& node )
	{
		rawPose.decode( node );

		return true;
	}

	Pose rawPose;

	Pose cameraPose;

	Pose footprintPose;

	Pose mapPose;
};

struct StargazerMapPoint
{
	StargazerMapPoint() :
		id(""),
		pose(),
		measurements()
	{}

	StargazerMapPoint(Pose pose) :
		id(id),
		pose(pose),
		measurements()
	{}

	bool decode( const YAML::Node& node )
	{
		id = node["id"].as<std::string>( );
		pose.decode( node["location"] );
		measurements = node["measurements"].as<std::vector<StargazerMeasurement>>( );

		return true;
	}

	std::string id;

	Pose pose;

	std::vector<StargazerMeasurement> measurements;
};

struct StargazerAnchor
{
	StargazerAnchor() :
        id(""),
        pose()
    {}

	StargazerAnchor(std::string id, Pose pose) :
        id(id),
        pose(pose),
        vecMapPoints()
    {}

	bool decode( const YAML::Node& node )
	{
		id = node["id"].as<std::string>( );
		pose.decode( node["location"] );
		pose.y = node["location"][1].as<double>( );
		pose.z = node["location"][2].as<double>( );
		pose.orientation = node["orientation"].as<double>( );

		return true;
	}

    std::string id;

    Pose pose;

    std::vector<StargazerMapPoint> vecMapPoints;
};


namespace YAML
{
template<>
struct convert<StargazerAnchor>
{
	static bool decode( const Node& node, StargazerAnchor& anchor )
	{
		return anchor.decode( node );
	}
};

template<>
struct convert<StargazerMeasurement>
{
	static bool decode( const Node& node, StargazerMeasurement& measurement )
	{
		return measurement.decode( node );
	}
};
}

namespace srs {

std::string g_strDataFile;

class StargazerTest : public ::testing::Test
{
public:

	StarGazerPointTransformer  		m_pointTransformer;

	std::vector<StargazerAnchor>	m_vecAnchors;

	tf::Transform					m_footprintTransform;

public:

	StargazerTest( ) :
		m_footprintTransform( tf::Quaternion::getIdentity( ), tf::Vector3( 0.39f, 0.064f, 0.0 ) )
	{

	}

	void SetUp( )
	{
		std::string strAnchors( g_strDataFile );
		strAnchors += "/anchors.yaml";

		std::string strCalibrationFile( g_strDataFile );
		strCalibrationFile += "/robot-config.yaml";

		m_pointTransformer.Load( "/map", m_footprintTransform, strAnchors, strCalibrationFile );
	}

	void TearDown( )
	{

	}

	~StargazerTest( )
	{

	}

	void LoadTestData( )
	{
		try
		{
			YAML::Node document = YAML::LoadFile( g_strDataFile );

			if( !document.IsNull( ) )
			{
				m_vecAnchors = document["anchors"].as<std::vector<StargazerAnchor>>( );
			}
			else
			{
				ROS_ERROR_STREAM( "Test data file not found: " << g_strDataFile );
			}
		}
		catch( const std::runtime_error& e )
		{
			ROS_ERROR_STREAM( "Could not parse yaml file for test data: " << g_strDataFile << " " << e.what( ) );
		}
	}

	double GetLeftHandAngle( double dfRightHandDegrees )
	{
		double dfLeftHandAngleInRadians = m_pointTransformer.ConvertToRightHandRule( dfRightHandDegrees * M_PI / 180 );

		return dfLeftHandAngleInRadians * 180 / M_PI;
	}

};

TEST_F( StargazerTest, TestTransforms )
{
	double dfAngleLeftDegrees = GetLeftHandAngle( -0.47000 );

	tf::Quaternion rotation = m_pointTransformer.ConvertAngle( dfAngleLeftDegrees );

	double dfAngleRightRadians = tf::getYaw( rotation );

	double dfAngleRightDegrees = dfAngleRightRadians * 180.0f / M_PI;

	tf::Vector3 cameraOffset = m_pointTransformer.GetCameraOffset( rotation );

	tf::Vector3 footprintOffset = m_pointTransformer.GetFootprintOffset( rotation );

	tf::Vector3 stargazerOffset = m_pointTransformer.GetStargazerOffset( -18.489, 77.349, 511.051 );

	tf::Vector3 totalCameraOffset = cameraOffset + stargazerOffset;

	ROS_DEBUG_NAMED( "StarGazerPointTransformer", "stargazer (%2.5f): %2.5f, %2.5f, %2.5f\n",
		tf::getYaw( rotation ), stargazerOffset.getX( ), stargazerOffset.getY( ), stargazerOffset.getZ( ) );

	ROS_DEBUG_NAMED( "StarGazerPointTransformer", "partial camera (%2.5f): %2.5f, %2.5f, %2.5f\n",
		tf::getYaw( rotation ), cameraOffset.getX( ), cameraOffset.getY( ), cameraOffset.getZ( ) );

	ROS_DEBUG_NAMED( "StarGazerPointTransformer", "total camera (%2.5f): %2.5f, %2.5f, %2.5f\n",
		tf::getYaw( rotation ), totalCameraOffset.getX( ), totalCameraOffset.getY( ), totalCameraOffset.getZ( ) );

	ROS_DEBUG_NAMED( "StarGazerPointTransformer", "partial footprint (%2.5f): %2.5f, %2.5f, %2.5f\n",
		tf::getYaw( rotation ), footprintOffset.getX( ), footprintOffset.getY( ), footprintOffset.getZ( ) );

	tf::Vector3 totalFootprintOffset = totalCameraOffset + footprintOffset;

	ROS_DEBUG_NAMED( "StarGazerPointTransformer", "total footprint (%2.5f): %2.5f, %2.5f, %2.5f\n",
		tf::getYaw( rotation ), totalFootprintOffset.getX( ), totalFootprintOffset.getY( ), totalFootprintOffset.getZ( ) );
}

TEST_F( StargazerTest, TestMapTransform )
{
	double dfAngleLeftDegrees = GetLeftHandAngle( -0.47000 );

	tf::Pose pose( tf::Quaternion::getIdentity( ) );

	if( m_pointTransformer.TransformPoint( 98, -18.489, 77.349, 511.051, dfAngleLeftDegrees, pose ) )
	{
		tf::Vector3 mapOffset = pose.getOrigin( );

		tf:: Quaternion mapRotation = pose.getRotation( );

		ROS_DEBUG_NAMED( "StarGazerPointTransformer", "map (%2.5f): %2.5f, %2.5f, %2.5f\n",
			tf::getYaw( mapRotation ), mapOffset.getX( ), mapOffset.getY( ), mapOffset.getZ( ) );
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

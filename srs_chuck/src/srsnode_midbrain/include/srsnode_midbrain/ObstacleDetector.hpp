/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MIDBRAIN_OBSTACLE_DETECTOR_HPP_
#define MIDBRAIN_OBSTACLE_DETECTOR_HPP_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/multi/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/multi/geometries/register/multi_polygon.hpp>
#include <fstream>
#include <iostream>
#include <srslib_framework/MsgPose.h>
#include <srslib_framework/MsgSolution.h>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionFactory.hpp>

namespace bg = boost::geometry;

class Point {
public:
    double x, y;
    Point():x(),y(){}
    Point(double x, double y):x(x),y(y){}

    Point& operator+=(const Point& rhs)
	{
    	return *this;
	}

    friend Point operator+(Point lhs, const Point& rhs)
	{
		lhs += rhs;
		return lhs;
	}

	friend std::ostream& operator<<( std::ostream& os, const Point& point )
    {
    	os << point.x << ", " << point.y;
    }

};

typedef std::vector<Point> Ring;
typedef bg::model::segment<Point> Segment;
typedef bg::model::polygon<Point> Polygon;

BOOST_GEOMETRY_REGISTER_POINT_2D(Point, double, bg::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_RING(Ring)

std::ostream& operator<<( std::ostream& os, const Segment& segment );
std::ostream& operator<<( std::ostream& os, const Ring& ring );

namespace srs
{

class ObstacleDetector
{
	typedef std::function<void()> ObstacleDetectedFn;

public:
	ObstacleDetector( double footprint );
	virtual ~ObstacleDetector( );

	void SetDetectionCallback( ObstacleDetectedFn obstacleDetectedCallback );

	void SetTestMode( bool testMode );
	bool GetTestMode( ) const;

	void SetDesiredVelocity( double linear, double angular );

	void SetActualVelocity( double linear, double angular );

	void SetPose( const srslib_framework::MsgPose::ConstPtr& pose );

	void SetSolution( const srslib_framework::MsgSolution::ConstPtr& solution );

	void SetThreshold( uint32_t depthThreshold );

	void ProcessScan( const sensor_msgs::LaserScan::ConstPtr& scan );

	double GetSafeDistance( double linearVelocity, double angularVelocity ) const;

	double GetFootprint( ) const;

	Ring GetDangerZone( ) const;

private:

	void AddPoseToPolygon( const Pose<>& pose, Polygon& polygon ) const;

	void UpdateDangerZone( );

	ObstacleDetectedFn				m_obstacleDetectedCallback;

	Pose<>							m_pose;

	Polygon							m_posePolygon;

	Solution<GridSolutionItem>		m_solution;

	Polygon							m_dangerZone;

	double							m_desiredLinearVelocity;

	double							m_desiredAngularVelocity;

	double							m_actualLinearVelocity;

	double							m_actualAngularVelocity;

	uint32_t						m_depthThreshold;

	double							m_footprint;

};

} /* namespace srs */

#endif /* MIDBRAIN_OBSTACLE_DETECTOR_HPP_ */

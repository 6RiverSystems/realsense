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
#include <fstream>
#include <iostream>
#include <srslib_framework/MsgPose.h>
#include <srslib_framework/MsgSolution.h>

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

typedef std::vector<Point> Polygon;
typedef bg::model::segment<Point> Segment;
typedef bg::model::multi_polygon<Polygon> MuliPolygon;

BOOST_GEOMETRY_REGISTER_POINT_2D(Point, double, bg::cs::cartesian, x, y)
BOOST_GEOMETRY_REGISTER_RING(Polygon)

std::ostream& operator<<( std::ostream& os, const Segment& segment );
std::ostream& operator<<( std::ostream& os, const Polygon& polygon );

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

	void SetVelocity( double linear, double angular );

	void SetPose( const srslib_framework::MsgPose::ConstPtr& pose );

	void SetSolution( const srslib_framework::MsgSolution::ConstPtr& solution );

	void SetThreshold( uint32_t depthThreshold );

	void ProcessScan( const sensor_msgs::LaserScan::ConstPtr& scan, bool isIrScan );

	double GetSafeDistance( double linearVelocity, double angularVelocity ) const;

	double GetFootprint( ) const;

	Polygon GetDangerZone( ) const;

private:

	void UpdateDangerZone( );

	ObstacleDetectedFn				m_obstacleDetectedCallback;

	srslib_framework::MsgPose		m_pose;

	srslib_framework::MsgSolution	m_solution;

	Polygon							m_dangerZone;

	double							m_linearVelocity;

	double							m_angularVelocity;

	uint32_t						m_depthThreshold;

	double							m_footprint;

};

} /* namespace srs */

#endif /* MIDBRAIN_OBSTACLE_DETECTOR_HPP_ */

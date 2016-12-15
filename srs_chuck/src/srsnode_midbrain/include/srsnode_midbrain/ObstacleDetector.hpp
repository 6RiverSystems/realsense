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
#include <srslib_framework/Pose.h>
#include <srslib_framework/MsgSolution.h>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionFactory.hpp>

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
	ObstacleDetector( double footprintWidth, double footprintLength );
	virtual ~ObstacleDetector( );

	void SetDetectionCallback( ObstacleDetectedFn obstacleDetectedCallback );

	void SetVelocity( double linear, double angular );

	void SetPose( const srslib_framework::Pose::ConstPtr& pose );

	void SetSolution( const srslib_framework::MsgSolution::ConstPtr& solution );

	void SetThreshold( uint32_t threshold );

	void ProcessScan( const sensor_msgs::LaserScan::ConstPtr& scan );

	double GetSafeDistance( double linearVelocity, double angularVelocity ) const;

	double GetFootprintWidth( ) const;

	double GetFootprintLength( ) const;

	Ring GetDangerZone( ) const;

private:

	void AddPoseToPolygon( const Pose<>& pose, Polygon& polygon, double width, double length ) const;

	void UpdateDangerZone( );

	ObstacleDetectedFn				m_obstacleDetectedCallback;

	Pose<>							m_pose;

	bool							m_poseValid;

	Polygon							m_footprintPolygon;

	Polygon							m_posePolygon;

	Solution<Grid2dSolutionItem>*		m_solution;

	Polygon							m_dangerZone;

	double							m_footprintWidth;

	double							m_footprintLength;

	double							m_linearVelocity;

	double							m_angularVelocity;

	uint32_t						m_threshold;

};

} /* namespace srs */

#endif /* MIDBRAIN_OBSTACLE_DETECTOR_HPP_ */


/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

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

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>


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

class HardStopReflex
{
public:
    HardStopReflex();
    virtual ~HardStopReflex();

    void setPose(Pose<> pose);
    void setLidarPose(Pose<> pose);

    void setLaserScan(const sensor_msgs::LaserScan& scan);

    void setVelocity(const Velocity<>& velocity)
    {
        latestVelocity_ = velocity;
    };

    bool checkHardStop();

    bool checkForClear();
    std::vector<Pose<>> getDangerZoneForDisplay() const;

    void setFootprint(const std::vector<Pose<>> footprint)
    {
        footprint_ = footprint;
    };

private:
    bool updateDangerZone();

    void addPoseToPolygon(Polygon& polygon, Pose<> pose, const std::vector<Pose<>>& footprint);

    Pose<> latestPose_ = Pose<>::INVALID;
    Velocity<> latestVelocity_ = Velocity<>::INVALID;
    Pose<> lidarPose_ = Pose<>::INVALID;

    double nominalDecelRate_ = 0.7;  // m/s^2
    double angularDecelRate_ = 1.0;  // r/s^2

    std::vector<Pose<>> footprint_;

    Ring laserScan_;
    Polygon dangerZone_;

    bool hardStopActivated_ = false;
    bool waitingForClear_ = false;
};

} /* namespace srs */


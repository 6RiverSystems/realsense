/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
using namespace std;

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>
#include <srslib_framework/planning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid2d/Grid2dSolutionItem.hpp>
#include <srslib_framework/planning/pathplanning/trajectory/Trajectory.hpp>
#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/robot_profile/RobotProfile.hpp>
using namespace srs;

namespace srs {

class SimpleTrajectoryGenerator
{
public:
    SimpleTrajectoryGenerator(MapStack* mapStack) :
        mapStack_(mapStack)
    {}

    void fromSolution(Solution<Grid2dSolutionItem>* solution)
    {
        trajectory_.clear();
        rotations_.clear();

        if (solution->empty())
        {
            return;
        }

        extractRotations(solution, rotations_);
        interpolatePath(rotations_);
    }

    void getTrajectory(Trajectory<>& trajectory)
    {
        trajectory = trajectory_;
    }

private:
    typedef Pose<> WaypointType;
    typedef vector<WaypointType> PathType;

    double calculateDirection(double from, double to)
    {
        return BasicMath::sgn<double>(to - from);
    }

    void extractRotations(Solution<Grid2dSolutionItem>* solution, PathType& rotations)
    {
        rotations.clear();

        // Push the first node of the solution as initial waypoint
        // only if it not a rotate
        if (solution->getStart().actionType != Grid2dSolutionItem::ROTATE)
        {
            rotations.push_back(solution->getStart().fromPose);
        }

        // Ignore all but the rotations, which will be used as waypoints
        for (auto solutionNode : *solution)
        {
            switch (solutionNode.actionType)
            {
                case Grid2dSolutionItem::MOVE:
                    break;

                case Grid2dSolutionItem::ROTATE:
                    rotations.push_back(solutionNode.toPose);
                    break;
                case Grid2dSolutionItem::NONE:
                    break;
            }
        }

        // Push the last node of the solution as final waypoint
        // only if it is not a rotate
        if (solution->getGoal().actionType != Grid2dSolutionItem::ROTATE)
        {
            rotations.push_back(solution->getGoal().toPose);
        }
    }

    void interpolatePath(PathType& rotations)
    {
        if (rotations.size() < 2)
        {
            return;
        }

        // Calculate how many segments are in the path
        int totalSegments = rotations.size() - 1;
        int segment = 1;

        auto fromWaypoint = rotations.begin();
        auto toWaypoint = fromWaypoint + 1;

        while (toWaypoint != rotations.end())
        {
            bool isFirstStretch = segment == 1;
            bool isLastStretch = segment == totalSegments;

            // Interpolate the trajectory between the two waypoints
            interpolateBetweenWaypoints(*fromWaypoint, *toWaypoint,
                isFirstStretch, isLastStretch);

            // Advance to the next segment
            segment++;
            fromWaypoint++;
            toWaypoint++;
        }

        // Make sure that the destination waypoint is there
        // with a 0 maximum velocity
        pushWaypoint(*fromWaypoint, 0.0);
    }

    void interpolateBetweenWaypoints(WaypointType fromWaypoint, WaypointType toWaypoint,
        bool isFirstStretch, bool isLastStretch)
    {
        // Calculate the direction of the motion
        double directionX = calculateDirection(fromWaypoint.x, toWaypoint.x);
        double directionY = calculateDirection(fromWaypoint.y, toWaypoint.y);

        double distanceFromStart = 0.0;
        double distanceToEnd = PoseMath::euclidean(fromWaypoint, toWaypoint);

        // Begin from the initial waypoint
        Pose<> waypoint = fromWaypoint;
        double currentMaxVelocity = 0;

        if (isFirstStretch)
        {
            pushWaypoint(waypoint, currentMaxVelocity);
        }

        double resolution = mapStack_->getLogicalMap()->getResolution();

        // Calculate the change in pose that depend on the
        // direction of motion and the specified spacing
        Pose<> deltaPose = Pose<>(
            directionX * resolution,
            directionY * resolution,
            0.0);

        while (distanceToEnd > resolution)
        {
            waypoint = PoseMath::add<double>(waypoint, deltaPose);
            pushWaypoint(waypoint, currentMaxVelocity);

            distanceToEnd = PoseMath::euclidean(waypoint, toWaypoint);
        }

        // Make sure that the destination waypoint is there
        pushWaypoint(toWaypoint, currentMaxVelocity);
    }

    void pushWaypoint(Pose<> waypoint, double maxVelocity)
    {
        TrajectoryAnnotation annotation;
        annotation.maxVelocity = maxVelocity;

        trajectory_.push_back(waypoint, annotation);
    }

    PathType rotations_;

    MapStack* mapStack_;

    Trajectory<> trajectory_;
};

} // namespace srs

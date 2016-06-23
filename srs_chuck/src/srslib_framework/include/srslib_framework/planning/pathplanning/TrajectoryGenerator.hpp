/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef TRAJECTORYGENERATOR_HPP_
#define TRAJECTORYGENERATOR_HPP_

#include <vector>
using namespace std;

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/RobotProfile.hpp>
using namespace srs;

namespace srs {

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(RobotProfile& robot) :
        robot_(robot)
    {}

    void fromSolution(Solution<Grid2d>& solution)
    {
        trajectory_.clear();
        originalPath_.clear();
        reducedPath_.clear();

        if (solution.empty())
        {
            return;
        }

        extractPath(solution, originalPath_);

        double spacing;
        calculateSpacing(originalPath_, reducedPath_, spacing);

        interpolatePath(reducedPath_, spacing);
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

    void calculateMinMaxMean(PathType& path, double& minD, double& maxD, double& meanD)
    {
        minD = numeric_limits<double>::max();
        maxD = numeric_limits<double>::min();
        meanD = 0;
        int meanCount = 0;

        Pose<> previousWaypoint = path.front();
        for (auto waypoint : path)
        {
            double d = PoseMath::euclidean(previousWaypoint, waypoint);
            if (d > 0)
            {
                minD = min(minD, d);
                maxD = max(maxD, d);

                meanD += d;
                meanCount++;
            }

            previousWaypoint = waypoint;
        }

        meanD /= meanCount;
    }

    void calculateSpacing(PathType& path, PathType& reducedPath, double& spacing)
    {
        double minDistance;
        double maxDistance;
        double meanDistance;

        calculateMinMaxMean(path, minDistance, maxDistance, meanDistance);

        double minimumSpacing = 0.0001 * meanDistance;
        double maximumSpacing = 0.005 * meanDistance;

        filterPath(path, minimumSpacing, reducedPath);

        calculateMinMaxMean(reducedPath, minDistance, maxDistance, meanDistance);

        maximumSpacing = min(maximumSpacing, minDistance);
        spacing = max(minimumSpacing, maximumSpacing);
    }

    void extractPath(Solution<Grid2d>& solution, PathType& path)
    {
        path.clear();

        // Push the first node of the solution as initial waypoint
        // only if it not a rotate
        if (solution.getStart().actionType != SolutionNode<Grid2d>::ROTATE)
        {
            path.push_back(solution.getStart().fromPose);
        }

        // Ignore all but the rotations, which will be used as waypoints
        for (auto solutionNode : solution)
        {
            switch (solutionNode.actionType)
            {
                case SolutionNode<Grid2d>::MOVE:
                    break;

                case SolutionNode<Grid2d>::ROTATE:
                    path.push_back(solutionNode.toPose);
                    break;
            }
        }

        // Push the last node of the solution as final waypoint
        // only if it is not a rotate
        if (solution.getGoal().actionType != SolutionNode<Grid2d>::ROTATE)
        {
            path.push_back(solution.getGoal().toPose);
        }
    }

    void filterPath(PathType& path, double minDistance, PathType& reducedPath)
    {
        Pose<> previousWaypoint = path.front();
        reducedPath.push_back(previousWaypoint);

        for (auto waypoint : path)
        {
            double d = PoseMath::euclidean(previousWaypoint, waypoint);
            if (d > minDistance)
            {
                reducedPath.push_back(waypoint);
            }

            previousWaypoint = waypoint;
        }
    }

    void interpolatePath(PathType& path, double spacing)
    {
        if (path.size() < 2)
        {
            return;
        }

        // Calculate how many segments are in the path
        int totalSegments = path.size() - 1;
        int segment = 1;

        auto fromWaypoint = path.begin();
        auto toWaypoint = fromWaypoint + 1;

        while (toWaypoint != path.end())
        {
            bool isFirstStretch = segment == 1;
            bool isLastStretch = segment == totalSegments;

            // Interpolate the trajectory between the two waypoints
            interpolateBetweenWaypoints(*fromWaypoint, *toWaypoint,
                spacing,
                isFirstStretch, isLastStretch);

            // Make sure that the destination waypoint is there
            // with a 0 maximum velocity
            if (isLastStretch)
            {
                pushWaypoint(*toWaypoint, 0.0);
            }

            // Advance to the next segment
            segment++;
            fromWaypoint++;
            toWaypoint++;
        }
    }

    void interpolateBetweenWaypoints(WaypointType fromWaypoint, WaypointType toWaypoint,
        double spacing,
        bool isFirstStretch, bool isLastStretch)
    {
        // Calculate the direction of the motion
        double directionX = calculateDirection(fromWaypoint.x, toWaypoint.x);
        double directionY = calculateDirection(fromWaypoint.y, toWaypoint.y);

        // Check which direction is applicable. If the trajectory between the two waypoints
        // moves along the x axis, the two Y coordinates should be roughly the same
        // (2mm difference)
        bool movingOnX = BasicMath::equal<double>(fromWaypoint.y, toWaypoint.y, 0.002);
        bool movingOnY = BasicMath::equal<double>(fromWaypoint.x, toWaypoint.x, 0.002);

        double distanceFromStart = 0.0;
        double distanceToEnd = PoseMath::euclidean(fromWaypoint, toWaypoint);

        double vCoasting = robot_.travelLinearVelocity;
        if  (distanceToEnd < robot_.smallStraightDistance)
        {
            vCoasting = robot_.travelTurningVelocity;
        }

        // Make sure that if the total distance between the two waypoints
        // is smaller than the specified value, the maximum velocity is forced
        // to be the turning velocity
        double v0Max = robot_.travelLinearVelocity;
        if (!isFirstStretch || distanceToEnd < robot_.smallStraightDistance)
        {
            v0Max = robot_.travelTurningVelocity;
        }

        double vfMax = robot_.travelLinearVelocity;
        if (!isLastStretch || distanceToEnd < robot_.smallStraightDistance)
        {
            vfMax = robot_.travelTurningVelocity;
        }

        // Begin from the initial waypoint
        Pose<> waypoint = fromWaypoint;
        double currentMaxVelocity = v0Max;

        pushWaypoint(waypoint, currentMaxVelocity);

        // Calculate the change in pose that depend on the
        // direction of motion and the specified spacing
        Pose<> deltaPose = Pose<>(
            movingOnX ? directionX * spacing : 0.0,
            movingOnY ? directionY * spacing : 0.0,
            0.0);

        while (distanceToEnd > robot_.goalReachedDistance)
        {
            waypoint = PoseMath::add<double>(waypoint, deltaPose);

            if (distanceFromStart < robot_.travelTurningZoneRadius)
            {
                // If this segment is not the first stretch and
                currentMaxVelocity = v0Max;
            }
            else if (distanceToEnd < robot_.travelTurningZoneRadius)
            {
                currentMaxVelocity = vfMax;
            }
            else
            {
                currentMaxVelocity = vCoasting;
            }

            pushWaypoint(waypoint, currentMaxVelocity);

            distanceFromStart = PoseMath::euclidean(waypoint, fromWaypoint);
            distanceToEnd = PoseMath::euclidean(waypoint, toWaypoint);
        }

        // Make sure that the destination waypoint is there
        pushWaypoint(toWaypoint, vfMax);
    }

    void pushWaypoint(Pose<> waypoint, double maxVelocity)
    {
        TrajectoryAnnotation annotation;
        annotation.maxVelocity = maxVelocity;

        trajectory_.push_back(waypoint, annotation);
    }

    PathType originalPath_;
    PathType reducedPath_;

    RobotProfile& robot_;

    Trajectory<> trajectory_;
};

} // namespace srs

#endif // TRAJECTORYGENERATOR_HPP_

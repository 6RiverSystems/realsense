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
    typedef Pose<> WaypointType;
    typedef vector<WaypointType> PathType;

    TrajectoryGenerator(RobotProfile& robot) :
        robot_(robot)
    {}

    void fromRotation(Pose<> pose0, double thetaf, double dT)
    {
        trajectory_.clear();
        path_.clear();
        pathReduced_.clear();

        if (BasicMath::fpEqual<double>(pose0.theta, thetaf))
        {
            return;
        }
        interpolateRotation(pose0, pose0.theta, thetaf, dT);
    }

    void fromSolution(Solution<Grid2d>& solution, double dT)
    {
        trajectory_.clear();
        path_.clear();
        pathReduced_.clear();

        if (solution.empty())
        {
            return;
        }

        findPath(solution, path_);
        double delta = calculateDelta();
        interpolatePath(pathReduced_, delta, dT);
    }

    void getTrajectory(Trajectory<>& trajectory)
    {
        trajectory = trajectory_;
    }

private:
    double calculateDelta()
    {
        double minDistance;
        double maxDistance;
        double meanDistance;

        calculateMinMaxMean(path_, minDistance, maxDistance, meanDistance);

        double minimumDelta = 0.0001 * meanDistance;
        double maximumDelta = 0.005 * meanDistance;

        filterPath(path_, minimumDelta, pathReduced_);

        calculateMinMaxMean(pathReduced_, minDistance, maxDistance, meanDistance);

        maximumDelta = min(maximumDelta, minDistance);
        return max(minimumDelta, maximumDelta);
    }

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

        Pose<> previousWaypoint = path_.front();
        for (auto waypoint : path_)
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

    void findPath(Solution<Grid2d>& solution, PathType& path)
    {
        path.clear();

        // Push the first node of the solution as initial waypoint
        path.push_back(solution.getStart().fromPose);

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
        path.push_back(solution.getGoal().toPose);
    }

    void interpolatePath(PathType& waypoints, double spacing, double dT)
    {
        if (waypoints.size() < 2)
        {
            return;
        }

        // Calculate how many segments are in the path
        int totalSegments = waypoints.size() - 1;
        int segment = 1;

        auto fromWaypoint = waypoints.begin();
        auto toWaypoint = fromWaypoint + 1;

        double v0 = 0.0;
        double vf = 0.0;
        while (toWaypoint != waypoints.end())
        {
            bool lastStretch = segment == totalSegments;

            // If the segment is the last stretch the final velocity will be 0
            // otherwise it will be the default velocity for curves
            vf = lastStretch ? 0.0 : robot_.travelCurvingVelocity;

            // Interpolate the trajectory between the two waypoints
            interpolateWaypoints(*fromWaypoint, *toWaypoint, v0, vf, spacing, dT);

            // The next initial velocity is equal to the final velocity of this segment
            v0 = vf;

            // Advance to the next segment
            segment++;
            fromWaypoint++;
            toWaypoint++;
        }
    }

    void interpolateRotation(Pose<> waypoint, double theta0, double thetaf, double dT)
    {
        // Calculate the change in velocity that can be achieved
        // with the specified travel acceleration and time interval
        double deltaVelocity = robot_.travelAngularVelocity * dT;

        // They keep track of the initial and final velocities
        // for the ramp up and the ramp down in the segment
        double coastV = robot_.travelAngularVelocity;

        // Measure the distance between the two orientations
        double d = AngleMath::normalizeAngleRad(thetaf - theta0);

        // Calculate how many midpoints between the two waypoints
        int totalMidpoints = ceil(abs(d) / deltaVelocity);

        double deltaAngle = d / totalMidpoints;

        // Calculate the points needed to ramp up and down the velocity
        // using the default coasting values
        int rampUpPoints = floor((coastV) / deltaVelocity) - 1;
        int rampDownPoints = floor((coastV) / deltaVelocity) + 1;

        // If the segment is not the last stretch, slow down earlier
        // to maintain the curving velocity
        int rampDownIndex = totalMidpoints - rampDownPoints;

        // If it is not possible, calculate the maximum velocity achievable
        if (rampUpPoints + rampDownPoints > totalMidpoints)
        {
            rampUpPoints = totalMidpoints / 2;
            rampDownPoints = totalMidpoints - rampUpPoints;
            rampDownIndex = rampDownPoints;

            // There is not enough space to go full travel speed. The
            // coasting velocity must be adjusted
            coastV = rampUpPoints * deltaVelocity;
        }

        // Current velocity
        double currentVelocity = 0.0;

        // Begin from the initial waypoint
        for (int p = 0; p < totalMidpoints; p++)
        {
            // Calculate the new angle
            waypoint.theta += deltaAngle;

            // Change the velocity based on where the middle point. If
            // no change is applied, the robot is coasting
            if (p < rampDownIndex)
            {
                currentVelocity += deltaVelocity;
            }
            else if (p >= rampDownIndex)
            {
                currentVelocity -= deltaVelocity;
            }

            // Saturate between the selected extremes (coast and final
            // velocity of the stretch)
            currentVelocity = BasicMath::saturate(currentVelocity, coastV, 0.0);

            // Add the middle point and the velocity command to the trajectory
            trajectory_.push_back(waypoint, Velocity<>(0.0, currentVelocity));
        }
    }

    void interpolateWaypoints(WaypointType fromWaypoint, WaypointType toWaypoint,
        double upV0, double downVf, double spacing, double dT)
    {
        // Calculate the change in velocity that can be achieved
        // with the specified travel acceleration and time interval
        double deltaVelocity = robot_.travelLinearAcceleration * dT;

        // They keep track of the initial and final velocities
        // for the ramp up and the ramp down in the segment
        double coastV = robot_.travelLinearVelocity;

        // Measure the distance between the two waypoints
        double d = PoseMath::euclidean(fromWaypoint, toWaypoint);

        // Calculate how many midpoints between the two waypoints
        int totalMidpoints = ceil(d / spacing);

        double deltaSpace = d / totalMidpoints;

        // Calculate the points needed to ramp up and down the velocity
        // using the default coasting values
        int rampUpPoints = floor((coastV - upV0) / deltaVelocity) - 1;
        int rampDownPoints = floor((coastV - downVf) / deltaVelocity) + 1;

        int slowZone = ceil(robot_.travelCurveZoneRadius / spacing) - 1;

        // If the segment is not the first stretch, keep the curving velocity
        // for a while before ramping up
        int rampUpIndex = -1;
        if (!BasicMath::fpEqual<double>(upV0, 0.0))
        {
            rampUpIndex += slowZone;
        }

        // If the segment is not the last stretch, slow down earlier
        // to maintain the curving velocity
        int rampDownIndex = totalMidpoints - rampDownPoints;
        if (!BasicMath::fpEqual<double>(downVf, 0.0) && rampDownIndex > slowZone)
        {
            rampDownIndex -= slowZone;
            rampDownIndex = BasicMath::saturate<int>(rampDownIndex, totalMidpoints, 0);
        }

        // If it is not possible, calculate the maximum velocity achievable
        if (rampUpPoints + rampDownPoints > totalMidpoints)
        {
            rampUpPoints = totalMidpoints / 2;
            rampDownPoints = totalMidpoints - rampUpPoints;
            rampDownIndex = rampDownPoints;

            // There is not enough space to go full travel speed. The
            // coasting velocity must be adjusted
            coastV = rampUpPoints * deltaVelocity;
        }

        // Current velocity
        double currentVelocity = upV0;

        // Calculate the direction of the motion
        double directionX = calculateDirection(fromWaypoint.x, toWaypoint.x);
        double directionY = calculateDirection(fromWaypoint.y, toWaypoint.y);

        // Check which direction is applicable
        bool movingOnX = BasicMath::fpEqual<double>(fromWaypoint.y, toWaypoint.y, 0.002);
        bool movingOnY = BasicMath::fpEqual<double>(fromWaypoint.x, toWaypoint.x, 0.002);

        // Begin from the initial waypoint
        Pose<> waypoint = fromWaypoint;
        for (int p = 0; p < totalMidpoints; p++)
        {
            // If the trajectory between the two waypoints moves along the x
            // axis, the two Y coordinates should be roughly the same (2mm difference)
            if (movingOnX)
            {
                // Advance in the direction of motion
                waypoint.x += directionX * deltaSpace;
            }
            else if (movingOnY)
            {
                // Advance in the direction of motion
                waypoint.y += directionY * deltaSpace;
            }
            else {
                // TODO: Handle situations where the robot moves in both directions?
                throw;
            }

            // Change the velocity based on where the middle point. If
            // no change is applied, the robot is coasting
            if (p > rampUpIndex && p < rampDownIndex)
            {
                currentVelocity += deltaVelocity;
            }
            else if (p >= rampDownIndex)
            {
                currentVelocity -= deltaVelocity;
            }

            // Saturate between the selected extremes (coast and final
            // velocity of the stretch)
            currentVelocity = BasicMath::saturate(currentVelocity, coastV, downVf);

            // Add the middle point and the velocity command to the trajectory
            trajectory_.push_back(waypoint, Velocity<>(currentVelocity, 0.0));
        }
    }

    PathType path_;
    PathType pathReduced_;

    RobotProfile& robot_;

    Trajectory<> trajectory_;
};

} // namespace srs

#endif // TRAJECTORYGENERATOR_HPP_

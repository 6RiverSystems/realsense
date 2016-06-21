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
    typedef Pose<> WaypointType;
    typedef vector<WaypointType> PathType;

    TrajectoryGenerator(RobotProfile& robot) :
        robot_(robot)
    {}

    void fromSolution(Solution<Grid2d>& solution, double dT)
    {
        trajectory_.clear();
        path_.clear();

        if (solution.empty())
        {
            return;
        }

        findPath(solution, path_);
        interpolatePath(path_, dT);
    }

    void getTrajectory(Trajectory<>& trajectory)
    {
        trajectory = trajectory_;
    }

private:
    double calculateDirection(double from, double to)
    {
        return BasicMath::sgn<double>(to - from);
    }

    void findPath(Solution<Grid2d>& solution, PathType& path)
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

        cout << "Path: " << endl;
        for (auto milestone : path)
        {
            cout << milestone << endl;
        }
        cout << endl;
    }

    void interpolatePath(PathType& waypoints, double dT)
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

        double v0 = 0.0; //robot_.minLinearVelocity;
        double vf = 0.0;
        while (toWaypoint != waypoints.end())
        {
            bool isFirstStretch = segment == 1;
            bool isLastStretch = segment == totalSegments;

            // If the segment is the last stretch the final velocity will be 0
            // otherwise it will be the default velocity for curves
            vf = isLastStretch ? 0.0 : robot_.travelCurvingVelocity;

            // Interpolate the trajectory between the two waypoints
            interpolateWaypoints(*fromWaypoint, *toWaypoint,
                v0, vf, dT,
                isFirstStretch, isLastStretch);

            // The next initial velocity is equal to the final velocity of this segment
            v0 = vf;

            // Advance to the next segment
            segment++;
            fromWaypoint++;
            toWaypoint++;
        }
    }

    void interpolateWaypoints(WaypointType fromWaypoint, WaypointType toWaypoint,
        double upV0, double downVf, double dT,
        bool isFirstStretch, bool isLastStretch)
    {
        // Calculate the change in velocity that can be achieved
        // with the specified travel acceleration and the specified spacing
        double deltaVelocity = robot_.travelLinearAcceleration * dT;

        // They keep track of the initial and final velocities
        // for the ramp up and the ramp down in the segment
        double coastV = robot_.travelLinearVelocity;

        // Calculate the direction of the motion
        double directionX = calculateDirection(fromWaypoint.x, toWaypoint.x);
        double directionY = calculateDirection(fromWaypoint.y, toWaypoint.y);

        // Check which direction is applicable
        bool movingOnX = BasicMath::equal<double>(fromWaypoint.y, toWaypoint.y, 0.002);
        bool movingOnY = BasicMath::equal<double>(fromWaypoint.x, toWaypoint.x, 0.002);

        // Measure the distance from the starting waypoint
        double distanceFromStart = 0.0;

        // Measure the distance to the ending waypoint
        double distanceToEnd = PoseMath::euclidean(fromWaypoint, toWaypoint);

        // Calculate the distance trod during ramp up and down
        double timeRampUp = (coastV - upV0) / robot_.travelLinearAcceleration;
        double rampUpDistance = 0.5 * (coastV + upV0) * timeRampUp;

        double timeRampDown = (coastV - downVf) / robot_.travelLinearAcceleration;
        double rampDownDistance = 0.5 * (coastV + downVf) * timeRampDown;

        if (distanceToEnd < rampUpDistance + rampDownDistance)
        {
            rampUpDistance = distanceToEnd / 2;
            rampDownDistance = rampUpDistance;
        }

        double initiateRampUp = 0;
        double initiateRampDown = distanceToEnd - rampDownDistance;

        // If the total distance is bigger than twice the
        // curve radius, then the reduced velocity on curves can be applied
        if (distanceToEnd > 2 * robot_.travelCurveZoneRadius)
        {
            if (!isFirstStretch)
            {
                initiateRampUp += robot_.travelCurveZoneRadius;
            }
            if (!isLastStretch)
            {
                initiateRampDown -= robot_.travelCurveZoneRadius;
            }
        }

        // Current velocity and delta space
        double updatedUpV0 = BasicMath::threshold<double>(upV0,
            robot_.minLinearVelocity, robot_.minLinearVelocity);
        double currentVelocity = updatedUpV0;

        // Begin from the initial waypoint
        Pose<> waypoint = fromWaypoint;
        while (distanceToEnd > 0)
        {
            trajectory_.push_back(waypoint, Velocity<>(currentVelocity, 0.0));

            cout << setw(6) << currentVelocity << "  " << waypoint << endl;

            // Calculate how much space the robot will advance in the time step
            double deltaSpace = currentVelocity * dT +
                0.5 * (robot_.travelLinearAcceleration * dT * dT);

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
            if (distanceFromStart >= initiateRampUp && distanceFromStart < initiateRampDown)
            {
                currentVelocity += deltaVelocity;

                // Saturate between the selected extremes (coast and initial
                // velocity of the stretch)
                currentVelocity = BasicMath::saturate<double>(currentVelocity, coastV, updatedUpV0);
            }
            else if (distanceFromStart >= initiateRampDown)
            {
                currentVelocity -= deltaVelocity;

                // Saturate between the selected extremes (coast and final
                // velocity of the stretch), and set it to 0.0
                // if it is below the minimum physical linear velocity
                currentVelocity = BasicMath::saturate<double>(currentVelocity, coastV, downVf);
                currentVelocity = BasicMath::threshold<double>(currentVelocity,
                    robot_.minLinearVelocity, 0.0);
            }

            distanceToEnd -= deltaSpace;
            distanceFromStart += deltaSpace;
        }

        // Make sure that the down final velocity is included in the
        // last stretch of the trajectory
        if (isLastStretch)
        {
            Trajectory<>::NodeType lastNode = trajectory_.back();
            if (!VelocityMath::equal(lastNode.second, Velocity<>(downVf, 0.0)))
            {
                trajectory_.push_back(waypoint, Velocity<>(downVf, 0.0));
                cout << setw(6) << downVf << "  " << waypoint << endl;
            }
        }
    }

    PathType path_;

    RobotProfile& robot_;

    Trajectory<> trajectory_;
};

} // namespace srs

#endif // TRAJECTORYGENERATOR_HPP_

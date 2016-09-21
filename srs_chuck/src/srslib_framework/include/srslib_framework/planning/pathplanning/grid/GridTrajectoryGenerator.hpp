/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef GRIDTRAJECTORYGENERATOR_HPP_
#define GRIDTRAJECTORYGENERATOR_HPP_

#include <vector>
using namespace std;

#include <srslib_framework/math/BasicMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/math/VelocityMath.hpp>

#include <srslib_framework/planning/pathplanning/Solution.hpp>
#include <srslib_framework/planning/pathplanning/grid/GridSolutionItem.hpp>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/Trajectory.hpp>
#include <srslib_framework/robotics/robot_profile/RobotProfile.hpp>
using namespace srs;

namespace srs {

class GridTrajectoryGenerator
{
public:
    GridTrajectoryGenerator(RobotProfile& robot) :
        robot_(robot)
    {}

    void fromSolution(Solution<GridSolutionItem>& solution)
    {
        trajectory_.clear();
        path_.clear();

        if (solution.empty())
        {
            return;
        }

        extractPath(solution, path_);
        interpolatePath(path_);
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

    void extractPath(Solution<GridSolutionItem>& solution, PathType& path)
    {
        path.clear();

        // Push the first node of the solution as initial waypoint
        // only if it not a rotate
        if (solution.getStart().actionType != GridSolutionItem::ROTATE)
        {
            path.push_back(solution.getStart().fromPose);
        }

        // Ignore all but the rotations, which will be used as waypoints
        for (auto solutionNode : solution)
        {
            switch (solutionNode.actionType)
            {
                case GridSolutionItem::MOVE:
                    break;

                case GridSolutionItem::ROTATE:
                    path.push_back(solutionNode.toPose);
                    break;
            }
        }

        // Push the last node of the solution as final waypoint
        // only if it is not a rotate
        if (solution.getGoal().actionType != GridSolutionItem::ROTATE)
        {
            path.push_back(solution.getGoal().toPose);
        }
    }

    void interpolatePath(PathType& path)
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

        // Check which direction is applicable. If the trajectory between the two waypoints
        // moves along the x axis, the two Y coordinates should be roughly the same
        // (2mm difference)
        bool movingOnX = BasicMath::equal<double>(fromWaypoint.y, toWaypoint.y, 0.002);
        bool movingOnY = BasicMath::equal<double>(fromWaypoint.x, toWaypoint.x, 0.002);

        double distanceFromStart = 0.0;
        double distanceToEnd = PoseMath::euclidean(fromWaypoint, toWaypoint);

        double vCoasting = robot_.pathFollowMaxLinearVelocity;
        if  (distanceToEnd < robot_.pathFollowSmallStraightDistance)
        {
            vCoasting = robot_.pathFollowTurningVelocity;
        }

        // Make sure that if the total distance between the two waypoints
        // is smaller than the specified value, the maximum velocity is forced
        // to be the turning velocity
        double v0Max = robot_.pathFollowMaxLinearVelocity;
        if (!isFirstStretch || distanceToEnd < robot_.pathFollowSmallStraightDistance)
        {
            v0Max = robot_.pathFollowTurningVelocity;
        }

        double vfMax = robot_.pathFollowMaxLinearVelocity;
        if (!isLastStretch || distanceToEnd < robot_.pathFollowSmallStraightDistance)
        {
            vfMax = robot_.pathFollowTurningVelocity;
        }

        // Begin from the initial waypoint
        Pose<> waypoint = fromWaypoint;
        double currentMaxVelocity = v0Max;

        if (isFirstStretch)
        {
            pushWaypoint(waypoint, currentMaxVelocity);
        }

        // Calculate the change in pose that depend on the
        // direction of motion and the specified spacing
        double spacing = robot_.pathFollowGoalReachedDistance / 5.0;
        Pose<> deltaPose = Pose<>(
            movingOnX ? directionX * spacing : 0.0,
            movingOnY ? directionY * spacing : 0.0,
            0.0);

        while (distanceToEnd > robot_.pathFollowGoalReachedDistance)
        {
            waypoint = PoseMath::add<double>(waypoint, deltaPose);

            if (distanceFromStart < robot_.pathFollowTurningZoneRadius)
            {
                // If this segment is not the first stretch and
                currentMaxVelocity = v0Max;
            }
            else if (distanceToEnd < robot_.pathFollowTurningZoneRadius)
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

    PathType path_;

    RobotProfile& robot_;

    Trajectory<> trajectory_;
};

} // namespace srs

#endif // GRIDTRAJECTORYGENERATOR_HPP_

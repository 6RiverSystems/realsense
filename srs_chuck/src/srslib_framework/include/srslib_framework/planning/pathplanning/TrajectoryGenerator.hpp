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
#include <srslib_framework/robotics/robot/RobotProfile.hpp>
using namespace srs;

namespace srs {

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(RobotProfile& robot) :
        robot_(robot),
        t_(0.0)
    {}

    void fromRamp(Pose<> pose, double v0, double vf, double acceleration)
    {
        trajectory_.clear();
        waypoints_.clear();
        filteredWaypoints_.clear();
        t_ = 0.0;

        if (BasicMath::fpEqual<double>(v0, vf))
        {
            return;
        }
    }

    void fromSolution(Solution<Grid2d>& solution)
    {
        trajectory_.clear();
        waypoints_.clear();
        filteredWaypoints_.clear();
        t_ = 0.0;

        if (solution.empty())
        {
            return;
        }

        findWaypoints(solution);

        double minDistance;
        double maxDistance;
        double meanDistance;

        calculateMinMaxMean(waypoints_, minDistance, maxDistance, meanDistance);

        double minimumDelta = 0.0001 * meanDistance;
        double maximumDelta = 0.005 * meanDistance;

        removeWaypoints(waypoints_, minimumDelta, filteredWaypoints_);

        calculateMinMaxMean(filteredWaypoints_, minDistance, maxDistance, meanDistance);

        maximumDelta = min(maximumDelta, minDistance);
        double delta = max(minimumDelta, maximumDelta);

        interpolateWaypoints(filteredWaypoints_, delta);
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

    void calculateMinMaxMean(vector<Pose<>>& waypoints,
        double& minDistance, double& maxDistance, double& meanDistance)
    {
        minDistance = numeric_limits<double>::max();
        maxDistance = numeric_limits<double>::min();
        meanDistance = 0;
        int meanCount = 0;

        Pose<> previousWaypoint = waypoints_.front();
        for (auto waypoint : waypoints_)
        {
            double d = PoseMath::euclidean(previousWaypoint, waypoint);
            if (d > 0)
            {
                meanDistance += d;
                meanCount++;

                if (d < minDistance)
                {
                    minDistance = d;
                }
                if (d > maxDistance)
                {
                    maxDistance = d;
                }
            }

            previousWaypoint = waypoint;
        }

        meanDistance /= meanCount;
    }

    void findWaypoints(Solution<Grid2d>& solution)
    {
        waypoints_.clear();

        for (auto node : solution)
        {
            switch (node.actionType)
            {
                case SolutionNode<Grid2d>::BACKWARD:
                case SolutionNode<Grid2d>::FORWARD:
                case SolutionNode<Grid2d>::ROTATE_180:
                    break;

                case SolutionNode<Grid2d>::START:
                case SolutionNode<Grid2d>::GOAL:
                case SolutionNode<Grid2d>::ROTATE_M90:
                case SolutionNode<Grid2d>::ROTATE_P90:
                    waypoints_.push_back(node.pose);
                    break;
            }
        }
    }

    void interpolateWaypoints(vector<Pose<>>& waypoints, double spacing)
    {
        if (waypoints.size() < 2)
        {
            return;
        }

        int segments = waypoints.size() - 1;

        auto fromWaypoint = waypoints.begin();
        auto toWaypoint = fromWaypoint + 1;
        Velocity<> currentVelocity;

        while (toWaypoint != waypoints.end())
        {
            currentVelocity = Velocity<>(robot_.travelLinearVelocity, 0.0);

            trajectory_.push_back(*fromWaypoint, currentVelocity);

            double d = PoseMath::euclidean(*fromWaypoint, *toWaypoint);
            int totalWaypoints = ceil(d / spacing) - 1;
            double delta  = d / totalWaypoints;

            if (BasicMath::fpEqual<double>(fromWaypoint->y, toWaypoint->y, 0.002))
            {
                // If the trajectory is moving along the x axis
                double direction = calculateDirection(fromWaypoint->x, toWaypoint->x);

                Pose<> waypoint = *fromWaypoint;
                for (int p = 0; p < totalWaypoints; p++)
                {
                    waypoint.x += direction * delta;
                    trajectory_.push_back(waypoint, currentVelocity);
                }
            }
            else if (BasicMath::fpEqual<double>(fromWaypoint->x, toWaypoint->x, 0.002))
            {
                // If the trajectory is moving along the y axis
                double direction = calculateDirection(fromWaypoint->y, toWaypoint->y);

                Pose<> waypoint = *fromWaypoint;
                for (int p = 0; p < totalWaypoints; p++)
                {
                    waypoint.y += direction * delta;
                    trajectory_.push_back(waypoint, currentVelocity);
                }
            }
            else
            {
                trajectory_.push_back(*fromWaypoint, currentVelocity);
            }

            fromWaypoint++;
            toWaypoint++;
        }
    }

    void removeWaypoints(vector<Pose<>>& waypoints, double minDistance, vector<Pose<>>& result)
    {
        Pose<> previousWaypoint = waypoints.front();
        result.push_back(previousWaypoint);

        for (auto waypoint : waypoints)
        {
            double d = PoseMath::euclidean(previousWaypoint, waypoint);
            if (d > minDistance)
            {
                result.push_back(waypoint);
            }

            previousWaypoint = waypoint;
        }
    }

    vector<Pose<>> filteredWaypoints_;

    RobotProfile& robot_;

    Trajectory<> trajectory_;
    double t_;

    vector<Pose<>> waypoints_;
};

} // namespace srs

#endif // TRAJECTORYGENERATOR_HPP_

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <vector>
using namespace std;

#include <srslib_framework/math/Math.hpp>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>

#include <srslib_framework/search/SolutionNode.hpp>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
#include <srslib_framework/robotics/robot/RobotProfile.hpp>
using namespace srs;

namespace srs {

class Trajectory
{
public:
    typedef pair<Pose<>, Velocity<>> MilestoneType;
    typedef vector<MilestoneType> TrajectoryType;

    Trajectory(RobotProfile& robot, double dT, double minVelocity = 0.3) :
        dT_(dT),
        linearIncrement_(dT * robot.linearAccelerationTravelMax()),
        minVelocity_(minVelocity),
        robot_(robot)
    {}

    void getTrajectory(vector<SolutionNode<Grid2d>>& solution, TrajectoryType& trajectory)
    {
        trajectory.clear();

        if (solution.empty())
        {
            return;
        }

        vector<Velocity<>> velocities;
        generateVelocities(solution, velocities);

        // TODO: Make sure that this pose and the orientation of the solution match in units (rad vs deg)
        SolutionNode<Grid2d> firstNode = solution[0];
        Pose<> pose0 = Pose<>(
            firstNode.action.position.location.x,
            firstNode.action.position.location.y,
            Math::deg2rad(firstNode.action.position.orientation));

        vector<Pose<>> poses;
        generatePoses(pose0, velocities, poses);

        for (unsigned int i = 0; i < velocities.size(); i++)
        {
            MilestoneType milestone;
            milestone.first = poses[i];
            milestone.second = velocities[i];
            trajectory.push_back(milestone);
        }
    }

    void setMinimumVelocity(double newValue)
    {
        minVelocity_ = newValue;
    }

private:
    void findNextAction(
        vector<SolutionNode<Grid2d>>::iterator& fromNode,
        vector<SolutionNode<Grid2d>>::iterator& resultNode)
    {
        resultNode = fromNode;
        vector<SolutionNode<Grid2d>>::iterator cursorNode = fromNode + 1;

        while (cursorNode->action.actionType == fromNode->action.actionType &&
            cursorNode->action.actionType != SearchAction<Grid2d>::GOAL)
        {
            resultNode = cursorNode;
            cursorNode++;
        }
    }

    void generatePoses(Pose<> pose0, vector<Velocity<>>& velocities, vector<Pose<>>& poses)
    {
        Pose<> previousPose = pose0;

        for (auto command : velocities)
        {
            double dT = command.arrivalTime - previousPose.arrivalTime;
            double x = previousPose.x + command.linear * dT * cos(previousPose.theta);
            double y = previousPose.y + command.linear * dT * sin(previousPose.theta);
            double theta = previousPose.theta + command.angular * dT;

            Pose<> newPose = Pose<>(previousPose.arrivalTime + dT, x, y, theta);
            poses.push_back(newPose);
            previousPose = newPose;
        }
    }

    void generateVelocities(vector<SolutionNode<Grid2d>>& solution, vector<Velocity<>>& velocities)
    {
        vector<SolutionNode<Grid2d>>::iterator fromNode = solution.begin();
        vector<SolutionNode<Grid2d>>::iterator node;
        vector<SolutionNode<Grid2d>>::iterator toNode;

        double v0 = 0.0;
        double vf = 0.0;

        SearchAction<Grid2d>::ActionEnum nextAction;
        bool lastAction = false;

        while (!lastAction)
        {
            node = fromNode + 1;
            nextAction = node->action.actionType;
            findNextAction(node, toNode);

            node = toNode + 1;
            lastAction = node->action.actionType == SearchAction<Grid2d>::GOAL;

            v0 = vf;
            vf = lastAction ? 0.0 : minVelocity_;

            switch (nextAction)
            {
                case SearchAction<Grid2d>::BACKWARD:
                    break;

                case SearchAction<Grid2d>::FORWARD:
                    moveForward(fromNode, v0, toNode, vf, velocities);
                    break;

                case SearchAction<Grid2d>::ROTATE_180:
                    break;

                case SearchAction<Grid2d>::ROTATE_M90:
                    break;

                case SearchAction<Grid2d>::ROTATE_P90:
                    break;
            }

            fromNode = toNode;
        }
    }

    double measureDistance(
        vector<SolutionNode<Grid2d>>::iterator& fromNode,
        vector<SolutionNode<Grid2d>>::iterator& toNode)
    {
        SearchPosition<Grid2d> fromPosition = fromNode->action.position;
        SearchPosition<Grid2d> toPosition = toNode->action.position;

        // TODO: Remember to multiply by the resolution of the map
        return Math::euclidean<double>(
            static_cast<double>(fromPosition.location.x),
            static_cast<double>(fromPosition.location.y),
            static_cast<double>(toPosition.location.x),
            static_cast<double>(toPosition.location.y));
    }

    void moveForward(
        vector<SolutionNode<Grid2d>>::iterator& fromNode, double initialV,
        vector<SolutionNode<Grid2d>>::iterator& toNode, double finalV,
        vector<Velocity<>>& velocities)
    {
        double ar = robot_.linearAccelerationTravelMax();

        double totalDistance = measureDistance(fromNode, toNode);

        // Assume that the travel velocity is maximum allowed
        double coastV = robot_.linearVelocityTravelMax();

        unsigned int intervals = calculateIntervals(coastV, ar);
        double rampDistance = 0.5 * intervals * dT_ * coastV;

        bool coasting = true;
        if (totalDistance < (2 * rampDistance))
        {
            // Recalculate the maximum achievable velocity
            // with the given acceleration
            coastV = sqrt(totalDistance * robot_.linearAccelerationTravelMax());
            intervals = calculateIntervals(coastV, ar);
            coasting = false;
        }

        double t = 0;

        // Acceleration
        vector<double> acceleration;
        // acceleration.push_back(initialV);
        generateRamp(initialV, coastV, intervals, acceleration);
        for (auto vt : acceleration)
        {
            velocities.push_back(Velocity<>(t, vt, 0));
            t += dT_;
        }

        // Coast
        if (coasting)
        {
            double tf = t + (totalDistance - 2 * rampDistance) / coastV;
            while (t < tf)
            {
                velocities.push_back(Velocity<>(t, coastV, 0));
                t += dT_;
            }
        }

        // Deceleration
        vector<double> deceleration;
        generateRamp(coastV, finalV, intervals, deceleration);
        for (auto vt : deceleration)
        {
            velocities.push_back(Velocity<>(t, vt, 0));
            t += dT_;
        }

        fromNode = toNode;
    }

    unsigned int calculateIntervals(double velocity, double ar)
    {
        return static_cast<unsigned int>(velocity / (dT_ * ar));
    }

    void generateRamp(double startValue, double endValue, unsigned int n, vector<double>& values)
    {
        double delta = (endValue - startValue) / n;

        for (unsigned int i = 1; i < n; i++)
        {
            double v = startValue + delta * i;
            v = abs(v) > 1e-4 ? v : 0.0;
            values.push_back(v);
        }
        values.push_back(endValue);
    }

    double dT_;

    double linearIncrement_;

    double minVelocity_;

    RobotProfile& robot_;
};

} // namespace srs

#endif // TRAJECTORY_HPP_

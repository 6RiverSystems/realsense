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

#include <srslib_framework/planning/pathplanning/SolutionNode.hpp>

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
        robot_(robot),
        t_(0.0)
    {}

    void calculateTrajectory(vector<SolutionNode<Grid2d>>& solution)
    {
        velocities_.clear();
        poses_.clear();

        if (solution.empty())
        {
            return;
        }

        vector<SolutionNode<Grid2d>>::iterator fromNode = solution.begin();
        vector<SolutionNode<Grid2d>>::iterator node;
        vector<SolutionNode<Grid2d>>::iterator toNode;

        double v0 = 0.0;
        double vf = 0.0;

        SolutionNode<Grid2d>::ActionEnum nextAction;
        bool lastAction = false;

        t_ = 0.0;
        while (!lastAction)
        {
            node = fromNode + 1;
            nextAction = node->actionType;
            findNextAction(node, toNode);

            node = toNode + 1;
            lastAction = node->actionType == SolutionNode<Grid2d>::GOAL;

            v0 = vf;
            vf = lastAction ? 0.0 : minVelocity_;

            switch (nextAction)
            {
                case SolutionNode<Grid2d>::BACKWARD:
                    break;

                case SolutionNode<Grid2d>::FORWARD:
                    moveForward(fromNode, v0, toNode, vf);
                    break;

                case SolutionNode<Grid2d>::ROTATE_180:
                case SolutionNode<Grid2d>::ROTATE_M90:
                case SolutionNode<Grid2d>::ROTATE_P90:
                    break;
            }

            fromNode = toNode;
        }
    }

    void getTrajectory(TrajectoryType& trajectory)
    {
        trajectory.clear();
        if (velocities_.empty())
        {
            return;
        }

        for (unsigned int i = 0; i < velocities_.size(); i++)
        {
            MilestoneType milestone;
            milestone.first = poses_[i];
            milestone.second = velocities_[i];
            trajectory.push_back(milestone);
        }
    }

    void setMinimumVelocity(double newValue)
    {
        minVelocity_ = newValue;
    }

private:
    Pose<> calculateNextPose(double dT, Pose<> previousPose, double vt)
    {
        double x = previousPose.x + vt * dT * cos(previousPose.theta);
        double y = previousPose.y + vt * dT * sin(previousPose.theta);
        double theta = previousPose.theta;

        return Pose<>(dT, x, y, theta);
    }

    void findNextAction(
        vector<SolutionNode<Grid2d>>::iterator& fromNode,
        vector<SolutionNode<Grid2d>>::iterator& resultNode)
    {
        resultNode = fromNode;
        vector<SolutionNode<Grid2d>>::iterator cursorNode = fromNode + 1;

        while (cursorNode->actionType == fromNode->actionType &&
            cursorNode->actionType != SolutionNode<Grid2d>::GOAL)
        {
            resultNode = cursorNode;
            cursorNode++;
        }
    }

    double measureDistance(
        vector<SolutionNode<Grid2d>>::iterator& fromNode,
        vector<SolutionNode<Grid2d>>::iterator& toNode)
    {
        Pose<> fromPose = fromNode->pose;
        Pose<> toPose = toNode->pose;

        // TODO: Add an euclidean for Pose
        return Math::euclidean<double>(
            static_cast<double>(fromPose.x),
            static_cast<double>(fromPose.y),
            static_cast<double>(toPose.x),
            static_cast<double>(toPose.y));
    }

    void moveForward(
        vector<SolutionNode<Grid2d>>::iterator& fromNode, double initialV,
        vector<SolutionNode<Grid2d>>::iterator& toNode, double finalV)
    {
        double forwardA = robot_.linearAccelerationTravelMax();
        double coastV = robot_.linearVelocityTravelMax();

        double totalDistance = measureDistance(fromNode, toNode);

        double tUp = (coastV - initialV) / forwardA;
        double rampDistanceUp = 0.5 * forwardA * tUp * tUp;

        double tDown = (coastV - finalV) / forwardA;
        double rampDistanceDown = 0.5 * forwardA * tDown * tDown;

        double ti = t_;

        bool coasting = true;
        if (rampDistanceUp + rampDistanceDown > totalDistance)
        {
            // Recalculate the maximum achievable velocity
            // with the given acceleration
        }

        Pose<> pose = Pose<>(dT_, fromNode->pose.x, fromNode->pose.y, fromNode->pose.theta);

        // Acceleration
        double vt = initialV;
        while (vt < coastV)
        {
            pose = calculateNextPose(dT_, pose, vt);
            poses_.push_back(pose);

            vt += forwardA * dT_;
            velocities_.push_back(Velocity<>(dT_, vt, 0));
            t_ += dT_;
        }

        // Coast
        if (coasting)
        {
            double tf = ti + tUp + (totalDistance - 2 * max(rampDistanceUp, rampDistanceDown)) / coastV;
            while (t_ < tf)
            {
                pose = calculateNextPose(dT_, pose, coastV);
                poses_.push_back(pose);

                velocities_.push_back(Velocity<>(dT_, coastV, 0));
                t_ += dT_;
            }
        }

        // Deceleration
        vt = coastV;
        while (vt > finalV)
        {
            pose = calculateNextPose(dT_, pose, vt);
            poses_.push_back(pose);

            vt -= forwardA * dT_;
            vt = abs(vt - finalV) > 1e-4 ? vt : 0.0;
            velocities_.push_back(Velocity<>(dT_, vt, 0));
            t_ += dT_;
        }

        fromNode = toNode;
    }

    double dT_;

    double linearIncrement_;

    double minVelocity_;

    vector<Pose<>> poses_;

    RobotProfile& robot_;

    double t_;

    vector<Velocity<>> velocities_;
};

} // namespace srs

#endif // TRAJECTORY_HPP_

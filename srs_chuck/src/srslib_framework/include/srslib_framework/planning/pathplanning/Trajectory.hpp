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
    Trajectory(vector<SolutionNode<Grid2d>>& solution, RobotProfile& robot, double dT) :
        solution_(solution),
        robot_(robot),
        dT_(dT),
        linearIncrement_(dT * robot.linearAccelerationTravelMax())
    {}

    void solution2velocity(vector<Velocity<>>& velocities)
    {
        velocities.clear();
        if (solution_.empty())
        {
            return;
        }

        vector<SolutionNode<Grid2d>>::iterator fromNode = solution_.begin();
        vector<SolutionNode<Grid2d>>::iterator nextNode;
        vector<SolutionNode<Grid2d>>::iterator toNode;

        while (fromNode != solution_.end())
        {
            nextNode = fromNode + 1;
            findNextAction(nextNode, toNode);

            switch (nextNode->action.actionType)
            {
                case SearchAction<Grid2d>::BACKWARD:
                    break;

                case SearchAction<Grid2d>::FORWARD:
                    moveForward(fromNode, toNode, velocities);
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

    double measure(vector<SolutionNode<Grid2d>>::iterator& fromNode,
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
        vector<SolutionNode<Grid2d>>::iterator& fromNode,
        vector<SolutionNode<Grid2d>>::iterator& toNode,
        vector<Velocity<>>& velocities)
    {
        double ar = robot_.linearAccelerationTravelMax();

        double totalDistance = measure(fromNode, toNode);

        // Assume that the travel velocity is maximum allowed
        double vf = robot_.linearVelocityTravelMax();
        unsigned int intervals = calculateIntervals(vf, ar);
        double rampDistance = 0.5 * intervals * dT_ * vf;

        bool coasting = true;
        if (totalDistance < (2 * rampDistance))
        {
            // Recalculate the maximum achievable velocity
            // with the given acceleration
            vf = sqrt(totalDistance * robot_.linearAccelerationTravelMax());
            intervals = calculateIntervals(vf, ar);
            coasting = false;
        }

        double t = 0;

        // Acceleration
        vector<double> acceleration;
        acceleration.push_back(0.0);
        generateRamp(0.0, vf, intervals, acceleration);
        for (auto vt : acceleration)
        {
            velocities.push_back(Velocity<>(t, vt, 0));
            t += dT_;
        }

        // Coast
        if (coasting)
        {
            t += (totalDistance - 2 * rampDistance) / vf;
        }

        // Deceleration
        vector<double> deceleration;
        generateRamp(vf, 0.0, intervals, deceleration);
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

    vector<SolutionNode<Grid2d>>& solution_;
    RobotProfile& robot_;
    double dT_;

    double linearIncrement_;
};

} // namespace srs

#endif // TRAJECTORY_HPP_


//static void solution2Pose(vector<SolutionNode<Grid2d>>& solution, vector<Pose<>>& poses)
//{
//    poses.clear();
//
//    // Remove the start solution node
//    auto node = solution.begin();
//    node++;
//
//    while (node != solution.end())
//    {
//        SearchPosition<Grid2d> position = node->action.position;
//
//        switch (node->action.actionType)
//        {
//            case SearchAction<Grid2d>::BACKWARD:
//            case SearchAction<Grid2d>::FORWARD:
//                poses.push_back(Pose<>(position.location.x, position.location.y, position.orientation));
//                break;
//
//            case SearchAction<Grid2d>::ROTATE_180:
//            case SearchAction<Grid2d>::ROTATE_M90:
//            case SearchAction<Grid2d>::ROTATE_P90:
//                break;
//        }
//
//        node++;
//    }
//}


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

    void solution2velocity(Velocity<> v0, vector<Velocity<>>& velocities)
    {
        velocities.clear();
        if (solution_.empty())
        {
            return;
        }

        vector<SolutionNode<Grid2d>>::iterator currentNode = solution_.begin();

        while (currentNode != solution_.end())
        {
            switch (currentNode->action.actionType)
            {
                case SearchAction<Grid2d>::BACKWARD:
                    break;

                case SearchAction<Grid2d>::FORWARD:
                    moveForward(currentNode, v0, velocities);
                    break;

                case SearchAction<Grid2d>::ROTATE_180:
                    break;

                case SearchAction<Grid2d>::ROTATE_M90:
                    break;

                case SearchAction<Grid2d>::ROTATE_P90:
                    break;

                case SearchAction<Grid2d>::START:
                    // Ignore the start action
                    break;
            }

            currentNode++;
        }
    }

private:
    void findNextAction(vector<SolutionNode<Grid2d>>::iterator& fromNode,
        vector<SolutionNode<Grid2d>>::iterator& toNode)
    {
        toNode = fromNode;
        SearchAction<Grid2d>::ActionEnum previousAction = toNode->action.actionType;
        toNode++;

        while (toNode->action.actionType == previousAction &&
            toNode->action.actionType != SearchAction<Grid2d>::GOAL)
        {
            previousAction = toNode->action.actionType;
            toNode++;
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

    void moveForward(vector<SolutionNode<Grid2d>>::iterator& fromNode,
        Velocity<>& v0,
        vector<Velocity<>>& velocities)
    {
        vector<SolutionNode<Grid2d>>::iterator toNode;
        findNextAction(fromNode, toNode);

        double distance = measure(fromNode, toNode);
        fromNode = toNode;

        double targetVelocity = robot_.linearVelocityTravelMax();
        double t = 0;
        double velocity = v0.linear;

        unsigned int accelerationIntervals = static_cast<unsigned int>(targetVelocity / linearIncrement_);

        if (distance > 0)
        {
            // Acceleration
            for (unsigned int interval = accelerationIntervals; interval > 0; interval--)
            {
                velocity += linearIncrement_;
                t += dT_;
                velocities.push_back(Velocity<>(t, velocity, 0));
            }

            double distanceTraveled = (targetVelocity * t) / 2;

            // Coast
            distance -= 2 * distanceTraveled;
            double deltaT = distance / targetVelocity;
            t += deltaT;

            // Deceleration
            for (unsigned int interval = accelerationIntervals; interval > 0; interval--)
            {
                velocity -= linearIncrement_;
                t += dT_;
                velocities.push_back(Velocity<>(t, velocity, 0));
            }
        }
        else
        {
            //
        }
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


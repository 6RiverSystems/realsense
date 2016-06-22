/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <vector>
#include <limits>
#include <iomanip>
using namespace std;

#include <srslib_framework/math/PoseMath.hpp>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
using namespace srs;

namespace srs {

struct TrajectoryAnnotation
{
    double maxVelocity;

    TrajectoryAnnotation() :
        maxVelocity(0.0)
    {}

    friend ostream& operator<<(ostream& stream, const TrajectoryAnnotation& annotation)
    {
        stream << "Annotation {" <<
            "maxVelocity: " << annotation.maxVelocity;

        return stream << "}";
    }
};

template<typename TYPE = double>
class Trajectory : public vector<pair<Pose<TYPE>, TrajectoryAnnotation>>
{
public:
    typedef TYPE BaseType;
    typedef pair<Pose<BaseType>, TrajectoryAnnotation> NodeType;

    Trajectory()
    {}

    int findClosestPose(Pose<BaseType> toPose, int fromIndex = 0, double searchDistance = -1)
    {
        int toIndex = this->size() - 1;
        if (searchDistance > -1)
        {
            toIndex = calculateIndexFromDistance(fromIndex, searchDistance);
        }

        BaseType minimum = numeric_limits<double>::max();
        int minimumIndex = toIndex;

        for (int i = fromIndex; i < toIndex; i++)
        {
            Pose<BaseType> current = this->at(i).first;
            BaseType distance = PoseMath::euclidean2(current, toPose);
            if (distance < minimum)
            {
                minimum = distance;
                minimumIndex = i;
            }
        }

        return minimumIndex;
    }

    int findWaypointAtDistance(int fromIndex, double distance)
    {
        return calculateIndexFromDistance(fromIndex, distance);
    }

    Pose<BaseType> getGoal()
    {
        // TODO: Implement exception?
        if (!this->empty())
        {
            return this->back().first;
        }

        return Pose<BaseType>();
    }

    Pose<BaseType> getPose(int position)
    {
        if (position >= 0 && position < this->size())
        {
            return this->at(position).first;
        }

        // TODO: An exception should be thrown
        return Pose<BaseType>();
    }

    TrajectoryAnnotation getAnnotation(int position)
    {
        if (position >= 0 && position < this->size())
        {
            NodeType node = this->at(position);
            return node.second;
        }

        // TODO: An exception should thrown
        return TrajectoryAnnotation();
    }

    void getWaypoint(int position, Pose<BaseType>& pose, TrajectoryAnnotation& annotation)
    {
        if (position >= 0 && position < this->size())
        {
            NodeType node = this[position];
            pose = node.first;
            annotation = node.second;

            return;
        }

        // TODO: An exception should be thrown
    }

    friend ostream& operator<<(ostream& stream, const Trajectory& trajectory)
    {
        int counter = 0;

        stream << "Trajectory {" << endl;
        for (auto waypoint : trajectory)
        {
            stream << setw(4) << counter++ << ": " <<
                waypoint.first << ", " << waypoint.second << endl;
        }

        return stream << "}";
    }

    void push_back(Pose<BaseType> pose, TrajectoryAnnotation annotation)
    {
        this->vector<NodeType>::push_back(NodeType(pose, annotation));
    }

private:
    int calculateIndexFromDistance(int fromIndex, double distance)
    {
        double distance2 = distance * distance;

        Pose<BaseType> fromPose = this->at(fromIndex).first;
        int toIndex = fromIndex;

        BaseType distanceToPose;
        do {
            Pose<> toPose = this->at(toIndex).first;
            distanceToPose = PoseMath::euclidean2(fromPose, toPose);

            toIndex++;

        } while (toIndex < this->size() && distanceToPose < distance2);

        return toIndex - 1;
    }
};

} // namespace srs

#endif // TRAJECTORY_HPP_

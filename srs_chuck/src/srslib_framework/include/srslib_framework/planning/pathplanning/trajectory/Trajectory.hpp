/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

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
    typedef pair<Pose<TYPE>, TrajectoryAnnotation> TrajectoryElementType;

    Trajectory()
    {}

    int findClosestPose(Pose<TYPE> toPose, int fromIndex = 0, double searchDistance = -1)
    {
        int toIndex = this->size() - 1;
        if (searchDistance > -1)
        {
            toIndex = calculateIndexFromDistance(fromIndex, searchDistance);
        }

        TYPE minimum = numeric_limits<double>::max();
        int minimumIndex = toIndex;

        for (int i = fromIndex; i < toIndex; i++)
        {
            Pose<TYPE> current = this->at(i).first;
            TYPE distance = PoseMath::euclidean2(current, toPose);
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

    Pose<TYPE> getGoal()
    {
        // TODO: Implement exception?
        if (!this->empty())
        {
            return this->back().first;
        }

        return Pose<TYPE>();
    }

    Pose<TYPE> getPose(int position)
    {
        if (position >= 0 && position < this->size())
        {
            return this->at(position).first;
        }

        // TODO: An exception should be thrown
        return Pose<TYPE>();
    }

    TrajectoryAnnotation getAnnotation(int position)
    {
        if (position >= 0 && position < this->size())
        {
            TrajectoryElementType node = this->at(position);
            return node.second;
        }

        // TODO: An exception should thrown
        return TrajectoryAnnotation();
    }

    void getWaypoint(int position, Pose<TYPE>& pose, TrajectoryAnnotation& annotation)
    {
        if (position >= 0 && position < this->size())
        {
            TrajectoryElementType node = this[position];
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

    void push_back(Pose<TYPE> pose, TrajectoryAnnotation annotation)
    {
        this->vector<TrajectoryElementType>::push_back(TrajectoryElementType(pose, annotation));
    }

private:
    int calculateIndexFromDistance(int fromIndex, double distance)
    {
        double distance2 = distance * distance;

        Pose<TYPE> fromPose = this->at(fromIndex).first;
        int toIndex = fromIndex;

        TYPE distanceToPose;
        do {
            Pose<> toPose = this->at(toIndex).first;
            distanceToPose = PoseMath::euclidean2(fromPose, toPose);

            toIndex++;

        } while (toIndex < this->size() && distanceToPose < distance2);

        return toIndex - 1;
    }
};

} // namespace srs

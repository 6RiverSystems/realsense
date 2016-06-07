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

#include <srslib_framework/math/Math.hpp>
#include <srslib_framework/math/PoseMath.hpp>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
using namespace srs;

namespace srs {

template<typename TYPE = double>
class Trajectory : public vector<pair<Pose<TYPE>, Velocity<TYPE>>>
{
public:
    typedef TYPE BaseType;
    typedef pair<Pose<BaseType>, Velocity<BaseType>> NodeType;

    Trajectory()
    {}

    int findClosestPose(Pose<BaseType> toPose, int fromIndex = 0, double searchDistance = -1)
    {
        int toIndex = this->size() - 1;
        if (searchDistance > -1)
        {
            toIndex = fromIndex + 2 * round(searchDistance / calculateTrajectoryStep());
            if (toIndex > this->size())
            {
                toIndex = this->size() - 1;
            }
        }

        BaseType minimum = numeric_limits<double>::max();
        int minimumIndex = -1;

        for (int i = fromIndex; i < toIndex; i++)
        {
            Pose<BaseType> current = this->at(i).first;
            BaseType distance = PoseMath::euclidean(current, toPose);
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
        int deltaTo = round(distance / calculateTrajectoryStep());
        int toIndex = fromIndex + deltaTo;
        if (toIndex > this->size())
        {
            toIndex = this->size() - 1;
        }

        return toIndex;
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

    Velocity<BaseType> getVelocity(int position)
    {
        if (position >= 0 && position < this->size())
        {
            NodeType node = this->at(position);
            return node.second;
        }

        // TODO: An exception should thrown
        return Velocity<BaseType>();
    }

    void getWaypoint(int position, Pose<BaseType>& pose, Velocity<BaseType>& velocity)
    {
        if (position >= 0 && position < this->size())
        {
            NodeType node = this[position];
            pose = node.first;
            velocity = node.second;

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
            stream << setw(4) << counter++ << ": " << waypoint.first << ", " << waypoint.second << endl;
        }

        return stream << "}";
    }

    void push_back(Pose<BaseType> pose, Velocity<BaseType> velocity)
    {
        this->vector<NodeType>::push_back(NodeType(pose, velocity));
    }

private:

    double calculateTrajectoryStep()
    {
        return PoseMath::euclidean(this->at(0).first, this->at(1).first);
    }
};

} // namespace srs

#endif // TRAJECTORY_HPP_

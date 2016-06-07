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

// TODO: Make Trajectory like Solution (descendant of a vector)
template<typename TYPE = double>
class Trajectory : public Object
{
public:
    typedef TYPE BaseType;

    Trajectory()
    {}

    void clear()
    {
        trajectory_.clear();
    }

    bool empty()
    {
        return trajectory_.empty();
    }

    int findClosestPose(Pose<BaseType> toPose, int fromIndex = 0, double searchDistance = -1)
    {
        int toIndex = trajectory_.size() - 1;
        if (searchDistance > -1)
        {
            toIndex = fromIndex + 2 * round(searchDistance / calculateTrajectoryStep());
            if (toIndex > trajectory_.size())
            {
                toIndex = trajectory_.size() - 1;
            }
        }

        BaseType minimum = numeric_limits<double>::max();
        int minimumIndex = -1;

        for (int i = fromIndex; i < toIndex; i++)
        {
            Pose<BaseType> current = trajectory_[i].first;
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
        if (toIndex > trajectory_.size())
        {
            toIndex = trajectory_.size() - 1;
        }

        return toIndex;
    }

    Pose<BaseType> getGoal()
    {
        // TODO: Implement exception?
        if (!trajectory_.empty())
        {
            return trajectory_.back().first;
        }

        return Pose<BaseType>();
    }

    Pose<BaseType> getPose(int position)
    {
        if (position >= 0 && position < trajectory_.size())
        {
            return trajectory_[position].first;
        }

        // TODO: An exception should be thrown
        return Pose<BaseType>();
    }

    Velocity<BaseType> getVelocity(int position)
    {
        if (position >= 0 && position < trajectory_.size())
        {
            NodeType node = trajectory_[position];
            Velocity<BaseType> v = node.second;
            cout << v << endl;

            return v;
        }

        // TODO: An exception should thrown
        return Velocity<BaseType>();
    }

    void getWaypoint(int position, Pose<BaseType>& pose, Velocity<BaseType>& velocity)
    {
        if (position >= 0 && position < trajectory_.size())
        {
            NodeType node = trajectory_[position];
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
        for (auto waypoint : trajectory.trajectory_)
        {
            stream << setw(4) << counter++ << ": " << waypoint.first << ", " << waypoint.second << endl;
        }

        return stream << "}";
    }

    void push_back(Pose<BaseType> pose, Velocity<BaseType> velocity)
    {
        trajectory_.push_back(NodeType(pose, velocity));
    }

    int size()
    {
        return trajectory_.size();
    }

private:
    typedef pair<Pose<>, Velocity<>> NodeType;

    double calculateTrajectoryStep()
    {
        return PoseMath::euclidean(trajectory_[0].first, trajectory_[1].first);
    }

    vector<NodeType> trajectory_;
};

} // namespace srs

#endif // TRAJECTORY_HPP_

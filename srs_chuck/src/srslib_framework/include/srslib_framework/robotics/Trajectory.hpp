/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef TRAJECTORY_HPP_
#define TRAJECTORY_HPP_

#include <vector>
#include <limits>
using namespace std;

#include <srslib_framework/math/Math.hpp>
#include <srslib_framework/math/PoseMath.hpp>

#include <srslib_framework/robotics/Pose.hpp>
#include <srslib_framework/robotics/Velocity.hpp>
using namespace srs;

namespace srs {

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

    int findClosestPose(Pose<BaseType> toPose, int fromIndex = 0, double distance = -1)
    {
        int toIndex = trajectory_.size() - 1;
        if (distance > -1)
        {
            toIndex = fromIndex + 2 * round(distance / calculateTrajectoryStep());
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

    bool getGoal(Pose<BaseType> goal)
    {
        if (trajectory_.empty())
        {
            return false;
        }

        goal = trajectory_.back().first;
        return true;
    }

    bool getPose(int position, Pose<BaseType>& pose)
    {
        if (position >= 0 && position < trajectory_.size())
        {
            NodeType node = trajectory_[position];
            pose = node.first;

            return true;
        }

        return false;
    }

    bool getVelocity(int position, Velocity<BaseType>& velocity)
    {
        if (position >= 0 && position < trajectory_.size())
        {
            NodeType node = trajectory_[position];
            velocity = node.second;

            return true;
        }

        return false;
    }

    bool getWaypoint(int position, Pose<BaseType>& pose, Velocity<BaseType>& velocity)
    {
        if (position >= 0 && position < trajectory_.size())
        {
            NodeType node = trajectory_[position];
            pose = node.first;
            velocity = node.second;

            return true;
        }

        return false;
    }

    friend ostream& operator<<(ostream& stream, const Trajectory& trajectory)
    {
        int counter = 0;

        stream << "Trajectory {" << endl;
        for (auto waypoint : trajectory.trajectory_)
        {
            stream << counter++ << ": " << waypoint.first << ", " << waypoint.second << endl;
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

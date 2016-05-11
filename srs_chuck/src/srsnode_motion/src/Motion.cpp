#include <srsnode_motion/Motion.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/search/SearchPosition.hpp>
#include <srslib_framework/search/SolutionNode.hpp>
#include <srslib_framework/planning/pathplanning/Trajectory.hpp>
#include <srslib_framework/robotics/robot/Chuck.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Motion::Motion(string nodeName) :
    rosNodeHandle_(nodeName),
    executionTime_(0.0),
    nextScheduled_(-1),
    tapPlan_(nodeName)
{
    pubCmdVel_ = rosNodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void Motion::run()
{
    tapPlan_.connectTap();

    double dT =  1.0 / REFRESH_RATE_HZ;

    ros::Rate refreshRate(REFRESH_RATE_HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        // If there is a new plan to execute
        if (tapPlan_.newDataAvailable())
        {
            cmdVel_.clear();

            Chuck chuck;
            vector<SolutionNode<Grid2d>> solution = tapPlan_.getGoalPlan();

            Trajectory trajectoryConverter(solution, chuck, dT);
            trajectoryConverter.solution2velocity(Velocity<>(), cmdVel_);

            for (auto pose : cmdVel_)
            {
                cout << pose << endl;
            }

            executionTime_ = 0.0;
            nextScheduled_ = 0;
        }

        if (nextScheduled_ > -1)
        {
            cout << "[" << executionTime_ << "] Waiting for " << cmdVel_[nextScheduled_].arrivalTime << endl;
            executionTime_ += dT;

            if (executionTime_ >= cmdVel_[nextScheduled_].arrivalTime)
            {
                cout << "Executing: " << cmdVel_[nextScheduled_] << endl;

                Velocity<> currentCmdVel = cmdVel_[nextScheduled_++];
                if (nextScheduled_ >= cmdVel_.size())
                {
                    nextScheduled_ = -1;
                }

                geometry_msgs::Twist message;

                message.linear.x = currentCmdVel.linear;
                message.linear.y = 0;
                message.linear.z = 0;
                message.angular.x = 0;
                message.angular.y = 0;
                message.angular.z = currentCmdVel.angular;

                pubCmdVel_.publish(message);
            }
        }

        refreshRate.sleep();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

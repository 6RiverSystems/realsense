/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_navigation/global_planner/PlannerUtilities.h>
#include <costmap_2d/cost_values.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <cmath>

namespace srs {

geometry_msgs::PoseStamped shiftGoalToMinima(geometry_msgs::PoseStamped goal_pose,
                                             const costmap_2d::Costmap2D& costmap, double max_shift)
{
    double goal_x = goal_pose.pose.position.x;
    double goal_y = goal_pose.pose.position.y;
    double goal_yaw = tf::getYaw(goal_pose.pose.orientation);
    // If the goal is not on the costmap, return the original goal
    unsigned int goal_mx = 0, goal_my = 0;
    if (!costmap.worldToMap(goal_x, goal_y, goal_mx, goal_my))
    {
        ROS_DEBUG("Goal is not on the map.  Cannot shift.");
        return goal_pose;
    }

    if (max_shift < 0)
    {
        ROS_DEBUG("Max shift is negative.  Cannot shift.");
        return goal_pose;
    }

    // Check to see if the cost at the goal is 0.  If so, return.
    unsigned char goal_cost = costmap.getCost(goal_mx, goal_my);
    if (goal_cost == 0)
    {
        ROS_DEBUG("Goal is already at a 0 cost location.  Not shifting.");
        return goal_pose;
    }

    // Shift the goal orthogonal to its heading to a local minima.
    double resolution = costmap.getResolution();
    double shift_vector_x = sin(goal_yaw);
    double shift_vector_y = -cos(goal_yaw);

    // For now, just find the absolute minimum of the search region
    double current_x = goal_x;
    double current_y = goal_y;
    unsigned char current_cost = goal_cost;
    double current_shift = max_shift;

    double wx = 0.0, wy = 0.0;
    unsigned int mx = 0, my = 0;


    for (double shift_sign : {-1, 1})
    {
        for (double abs_shift = 0; abs_shift <= max_shift; abs_shift += resolution)
        {
            double shift = abs_shift * shift_sign;

            wx = goal_x + shift * shift_vector_x;
            wy = goal_y + shift * shift_vector_y;
            if (!costmap.worldToMap(wx, wy, mx, my))
            {
                ROS_INFO("Shift check goes off of the map");
                continue;
            }
            unsigned char cost = costmap.getCost(mx, my);
            ROS_DEBUG("Got cost %d at shift %f at [%f, %f] (%d, %d)", cost, shift, wx, wy, mx, my);
            if (cost == costmap_2d::LETHAL_OBSTACLE)
            {
                ROS_DEBUG("Can't shift goal through obstacle");
                break;
            }

            if (cost < current_cost
                || (cost < goal_cost && cost == current_cost && std::fabs(shift) < std::fabs(current_shift)))
            {
                current_shift = shift;
                current_cost = cost;
                current_x = wx;
                current_y = wy;
            }
        }

    }

    ROS_INFO("Updated goal position from (%f, %f, %f: %d) to (%f, %f, %f: %d)",
        goal_x, goal_y, goal_yaw, goal_cost, current_x, current_y, goal_yaw, current_cost);

   geometry_msgs::PoseStamped output = goal_pose;
   output.pose.position.x = current_x;
   output.pose.position.y = current_y;

   return output;
}

}
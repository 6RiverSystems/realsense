/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <gtest/gtest.h>

#include <srsnode_navigation/global_planner/PlannerUtilities.h>

#include <tf/tf.h>

typedef geometry_msgs::PoseStamped gmPS;

costmap_2d::Costmap2D* createTestCostmap2d()
{
  // Create a 20x20 costmap with 0.05 m resolution
  costmap_2d::Costmap2D* cm = new costmap_2d::Costmap2D(20, 20, 0.05, 0, 0, 0);

  // Put in a fixed set of data.
  unsigned char data[400] = {
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0,
    2, 1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 254, 254, 254, 254, 10, 9, 0
  };

  memcpy(cm->getCharMap(), data, sizeof(unsigned char) * 400);

  return cm;
}

gmPS createPoseStamped(float x, float y, float yaw)
{
  gmPS p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  return p;
}

bool poseStampedEqual(gmPS p1, gmPS p2)
{
  double epsilon = 0.001;

  double angle_diff = tf::getYaw(p1.pose.orientation) - tf::getYaw(p2.pose.orientation);
  while (angle_diff > M_PI)
  {
    angle_diff -= 2 * M_PI;
  }
  while (angle_diff < -M_PI)
  {
    angle_diff += 2 * M_PI;
  }
  return std::fabs(p1.pose.position.x - p2.pose.position.x) < epsilon
        && std::fabs(p1.pose.position.y - p2.pose.position.y) < epsilon
        && std::fabs(angle_diff) < epsilon;

}


TEST(Test_Navigation_Utilities, goal_shift)
{
  costmap_2d::Costmap2D* cm = createTestCostmap2d();

  std::cerr << "CM Check: " << (unsigned int)cm->getCost(0, 0) << ", " << (unsigned int) cm->getCost(10, 10) << std::endl;

  double max_shift = 0.2;

  // Goal is not on map.
  gmPS goal = createPoseStamped(-0.05, 0, 0);
  EXPECT_TRUE(poseStampedEqual(goal, srs::shiftGoalToMinima(goal, *cm, max_shift)));

  // Goal is already in the 0 trough
  goal = createPoseStamped(0.125, 0.55, M_PI / 2);
  EXPECT_TRUE(poseStampedEqual(goal, srs::shiftGoalToMinima(goal, *cm, max_shift)));

  // Goal is off the trough.
  goal = createPoseStamped(0.225, 0.55, M_PI / 2);
  EXPECT_TRUE(poseStampedEqual(createPoseStamped(0.125, 0.55, M_PI / 2),
    srs::shiftGoalToMinima(goal, *cm, max_shift)));

  // Goal is too far to get to the trough
  goal = createPoseStamped(0.425, 0.55, M_PI / 2);
  EXPECT_TRUE(poseStampedEqual(createPoseStamped(0.225, 0.55, M_PI / 2),
    srs::shiftGoalToMinima(goal, *cm, max_shift)));

  // Goal is off the trough but orthogonal to it.
  goal = createPoseStamped(0.225, 0.55, 0);
  EXPECT_TRUE(poseStampedEqual(createPoseStamped(0.225, 0.55, 0),
    srs::shiftGoalToMinima(goal, *cm, max_shift)));

  // Shift is set to zero
  goal = createPoseStamped(0.225, 0.55, M_PI / 2);
  EXPECT_TRUE(poseStampedEqual(createPoseStamped(0.225, 0.55, M_PI / 2),
    srs::shiftGoalToMinima(goal, *cm, 0.0)));
  delete cm;
}

// int main(int argc, char** argv)
// {
//   testing::InitGoogleTest( &argc, argv );
//   return RUN_ALL_TESTS();
// }


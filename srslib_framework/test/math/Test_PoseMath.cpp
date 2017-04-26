/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/math/PoseMath.hpp>
using namespace srs;

void testPose(Pose<> original, Pose<> correct)
{
    ASSERT_NEAR(correct.x, original.x, 0.001) << "X coordinate of pose incorrect";
    ASSERT_NEAR(correct.y, original.y, 0.001) << "Y coordinate of pose incorrect";
    ASSERT_NEAR(correct.theta, original.theta, 0.001) << "Theta angle of pose incorrect";
}

TEST(Test_PoseMath, Pose2Polygon_Forward)
{
    Pose<> center(1.0, 1.0, 0.0);
    vector<Pose<>> polygon = PoseMath::pose2Polygon(center, 1.0, 0.0, 2.0, 2.0);

    testPose(polygon[0], Pose<>(3, 0, 0));
    testPose(polygon[1], Pose<>(3, 2, 0));
    testPose(polygon[2], Pose<>(1, 2, 0));
    testPose(polygon[3], Pose<>(1, 0, 0));
}

TEST(Test_PoseMath, Pose2Polygon_Sideway)
{
    Pose<> center(1.0, 1.0, 0.0);
    vector<Pose<>> polygon = PoseMath::pose2Polygon(center, 0.0, 1.0, 2.0, 2.0);

    testPose(polygon[0], Pose<>(2, 1, 0));
    testPose(polygon[1], Pose<>(2, 3, 0));
    testPose(polygon[2], Pose<>(0, 3, 0));
    testPose(polygon[3], Pose<>(0, 1, 0));
}

TEST(Test_PoseMath, Pose2Polygon)
{
    Pose<> center(1.0, 1.0, 0.0);
    vector<Pose<>> polygon = PoseMath::pose2Polygon(center, 0.3, 0.7, 2.0, 1.0);

    testPose(polygon[0], Pose<>(1.8, 0.7, 0));
    testPose(polygon[1], Pose<>(1.8, 2.7, 0));
    testPose(polygon[2], Pose<>(0.8, 2.7, 0));
    testPose(polygon[3], Pose<>(0.8, 0.7, 0));
}

TEST(Test_PoseMath, Pose2Polygon45)
{
    Pose<> center(1.0, 1.0, AngleMath::deg2Rad<double>(45));
    vector<Pose<>> polygon = PoseMath::pose2Polygon(center, 1.0, 0.0, 2.0, 2.0);

    testPose(polygon[0], Pose<>(3.121, 1.707, 0.785));
    testPose(polygon[1], Pose<>(1.707, 3.121, 0.785));
    testPose(polygon[2], Pose<>(0.292, 1.707, 0.785));
    testPose(polygon[3], Pose<>(1.707, 0.292, 0.785));
}

TEST(Test_PoseMath, PoseMultiply)
{
    Pose<> robotInMap(1, 1, 0);
    Pose<> lidarOnRobot(1, 0, 0);
    testPose(PoseMath::multiply(robotInMap, lidarOnRobot), Pose<>(2, 1, 0));

    robotInMap = Pose<>(0, 0, M_PI/2);
    testPose(PoseMath::multiply(robotInMap, lidarOnRobot), Pose<>(0, 1, M_PI/2));
}
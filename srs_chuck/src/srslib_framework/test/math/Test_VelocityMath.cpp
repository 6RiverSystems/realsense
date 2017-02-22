/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/math/VelocityMath.hpp>
using namespace srs;

TEST(Test_VelocityMath, GreaterThanOr)
{
    Velocity<> velocity1(1, 1, 0);
    Velocity<> velocity2(0.1, 0, 0);

    ASSERT_TRUE(VelocityMath::greaterThanOr(velocity1, Velocity<>::ZERO)) <<
        "The comparison result is not as expected";

    ASSERT_FALSE(VelocityMath::greaterThanOr(velocity1, Velocity<>::INVALID)) <<
        "The comparison result is not as expected";

    ASSERT_TRUE(VelocityMath::greaterThanOr(velocity1, velocity2)) <<
        "The comparison result is not as expected";

    Velocity<> velocity3(0, 1, 0);
    ASSERT_FALSE(VelocityMath::greaterThanOr(velocity1, velocity3)) <<
        "The comparison result is not as expected";

    Velocity<> velocity4(0, 3, 0);
    ASSERT_FALSE(VelocityMath::greaterThanOr(velocity1, velocity4)) <<
        "The comparison result is not as expected";

    Velocity<> velocity5(4, 1, 0);
    ASSERT_FALSE(VelocityMath::greaterThanOr(velocity1, velocity5)) <<
        "The comparison result is not as expected";

    Velocity<> velocity6(1, 4, 0);
    ASSERT_FALSE(VelocityMath::greaterThanOr(velocity1, velocity6)) <<
        "The comparison result is not as expected";
}

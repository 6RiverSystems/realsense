/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <limits>
using namespace std;

#include <srslib_framework/math/BasicMath.hpp>
using namespace srs;

TEST(Test_BasicMath, Integer_NoOverflowAdd_PositiveA_PositiveB)
{
    ASSERT_EQ(4, BasicMath::noOverflowAdd<int>(2, 2));

    ASSERT_EQ(1002, BasicMath::noOverflowAdd<int>(1000, 2));

    ASSERT_EQ(numeric_limits<int>::max(),
        BasicMath::noOverflowAdd<int>(1000, numeric_limits<int>::max()));

    ASSERT_EQ(numeric_limits<int>::max(),
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::max(), 1000));

    ASSERT_EQ(numeric_limits<int>::max(),
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::max(), numeric_limits<int>::max()));

    ASSERT_EQ(numeric_limits<int>::max(),
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::max(), numeric_limits<int>::max() - 10));
}

TEST(Test_BasicMath, Integer_NoOverflowAdd_PositiveA_NegativeB)
{
    ASSERT_EQ(0, BasicMath::noOverflowAdd<int>(2, -2));

    ASSERT_EQ(998, BasicMath::noOverflowAdd<int>(1000, -2));

    ASSERT_EQ(numeric_limits<int>::min() + 1000,
        BasicMath::noOverflowAdd<int>(1000, numeric_limits<int>::min()));

    ASSERT_EQ(numeric_limits<int>::min(),
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::min(), -1000));

    ASSERT_EQ(numeric_limits<int>::min(),
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::min(), numeric_limits<int>::min()));

    ASSERT_EQ(numeric_limits<int>::min(),
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::min(), numeric_limits<int>::min() + 10));
}

TEST(Test_BasicMath, Integer_NoOverflowAdd_NegativeA_PositiveB)
{
    ASSERT_EQ(0, BasicMath::noOverflowAdd<int>(-2, 2));

    ASSERT_EQ(-998, BasicMath::noOverflowAdd<int>(-1000, 2));

    ASSERT_EQ(numeric_limits<int>::max() - 1000,
        BasicMath::noOverflowAdd<int>(-1000, numeric_limits<int>::max()));

    ASSERT_EQ(numeric_limits<int>::min() + 1000,
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::min(), 1000));

    ASSERT_EQ(-1,
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::min(), numeric_limits<int>::max()));

    ASSERT_EQ(-11,
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::min(), numeric_limits<int>::max() - 10));
}

TEST(Test_BasicMath, Integer_NoOverflowAdd_NegativeA_NegativeB)
{
    ASSERT_EQ(-4, BasicMath::noOverflowAdd<int>(-2, -2));

    ASSERT_EQ(-1002, BasicMath::noOverflowAdd<int>(-1000, -2));

    ASSERT_EQ(numeric_limits<int>::min(),
        BasicMath::noOverflowAdd<int>(-1000, numeric_limits<int>::min()));

    ASSERT_EQ(numeric_limits<int>::min(),
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::min(), -1000));

    ASSERT_EQ(numeric_limits<int>::min(),
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::min(), numeric_limits<int>::min()));

    ASSERT_EQ(-10,
        BasicMath::noOverflowAdd<int>(numeric_limits<int>::min(), numeric_limits<int>::min() - 10));
}

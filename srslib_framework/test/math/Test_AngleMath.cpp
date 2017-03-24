/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <limits>
using namespace std;

#include <srslib_framework/math/AngleMath.hpp>
using namespace srs;

TEST(Test_AngleMath, NormalizeDeg)
{
    ASSERT_EQ(10, AngleMath::normalizeDeg<int>(10));
    ASSERT_EQ(340, AngleMath::normalizeDeg<int>(-20));
    ASSERT_EQ(0, AngleMath::normalizeDeg<int>(0));
    ASSERT_EQ(185, AngleMath::normalizeDeg<int>(185));
    ASSERT_EQ(160, AngleMath::normalizeDeg<int>(-200));

    ASSERT_DOUBLE_EQ(10.0, AngleMath::normalizeDeg<double>(10));
    ASSERT_DOUBLE_EQ(340, AngleMath::normalizeDeg<double>(-20));
    ASSERT_DOUBLE_EQ(0, AngleMath::normalizeDeg<double>(0));
    ASSERT_DOUBLE_EQ(185.0, AngleMath::normalizeDeg<double>(185));
    ASSERT_DOUBLE_EQ(160, AngleMath::normalizeDeg<double>(-200));

    ASSERT_DOUBLE_EQ(10.0, AngleMath::normalizeDeg<float>(10));
    ASSERT_DOUBLE_EQ(340.0, AngleMath::normalizeDeg<float>(-20));
    ASSERT_DOUBLE_EQ(0.0, AngleMath::normalizeDeg<float>(0));
    ASSERT_DOUBLE_EQ(185.0, AngleMath::normalizeDeg<float>(185));
    ASSERT_DOUBLE_EQ(160, AngleMath::normalizeDeg<float>(-200));
}


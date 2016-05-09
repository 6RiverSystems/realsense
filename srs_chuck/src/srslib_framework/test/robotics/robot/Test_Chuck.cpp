/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <srslib_framework/robotics/robot/Chuck.hpp>
using namespace srs;

TEST(Test_Chuck, Usage)
{
    Chuck robotProfile;
    cout << robotProfile.width() << endl;
}

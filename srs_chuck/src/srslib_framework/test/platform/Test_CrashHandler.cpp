/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <string>
using namespace std;

TEST(Test_Platform, CrashHandler)
{
    char* n = nullptr;
    string s(n);
}

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/localization/map/mapnote/MapNote2.hpp>
#include <srslib_framework/localization/map/mapnote/MapNotes.hpp>

using namespace srs;

TEST(Test_MapNote, Basic)
{
    MapNote2* note = MapNote2::instanceOf(WARNING_SOUND);

}

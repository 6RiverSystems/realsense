/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#include <gtest/gtest.h>

#include <vector>
using namespace std;

#include <ros/ros.h>

#include <srslib_framework/localization/map/mapnote/MapNote.hpp>
#include <srslib_framework/localization/map/mapnote/MapNotes.hpp>
#include <srslib_framework/localization/map/mapnote/NotePlaySound.hpp>
#include <srslib_framework/localization/map/mapnote/NoteSetMaxVelocity.hpp>

using namespace srs;

TEST(Test_MapNotes, Basic)
{
    MapNotes notes;
    notes.add(MapNote::PLAY_SOUND, "warning");
    notes.add(MapNote::instanceOf(MapNote::SET_MAX_VELOCITY, "22"));

    cout << notes << endl;

    ASSERT_FALSE(notes.add(MapNote::SET_MAX_VELOCITY, "33")) <<
        "The note was added regardless being a duplicate";

    ASSERT_TRUE(notes.has(MapNote::PLAY_SOUND)) <<
        "The sound note was not found";

    ASSERT_TRUE(notes.has(MapNote::SET_MAX_VELOCITY)) <<
        "The max_velocity note was not found";

    ASSERT_FALSE(notes.has(MapNote::NONE)) <<
        "The none note was found";

    shared_ptr<NoteSetMaxVelocity> maxVelocity = notes.get<NoteSetMaxVelocity>(MapNote::SET_MAX_VELOCITY);

    ASSERT_FLOAT_EQ(22.0, maxVelocity->getMaxVelocity()) <<
        "The max velocity is not as expected";
}

TEST(Test_MapNotes, Equal)
{
    MapNotes notes1;
    notes1.add(MapNote::PLAY_SOUND, "warning");
    notes1.add(MapNote::instanceOf(MapNote::SET_MAX_VELOCITY, "22"));

    MapNotes notes2;
    notes2.add(MapNote::PLAY_SOUND, "warning");
    notes2.add(MapNote::instanceOf(MapNote::SET_MAX_VELOCITY, "22"));

    ASSERT_EQ(notes1, notes2) <<
        "The notes are expected to be equal";

    MapNotes notes3;
    notes3.add(MapNote::PLAY_SOUND, "warning");

    ASSERT_NE(notes1, notes3) <<
        "The notes are not expected to be equal";
}

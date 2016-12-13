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
#include <srslib_framework/localization/map/mapnote/MapNoteFactory.hpp>
#include <srslib_framework/localization/map/mapnote/MapNotes.hpp>
#include <srslib_framework/localization/map/mapnote/NotePlaySound.hpp>
#include <srslib_framework/localization/map/mapnote/NoteSetMaxVelocity.hpp>

using namespace srs;

TEST(Test_MapNotes, Basic)
{
    MapNotes notes;
    notes.add(NotePlaySound::TYPE, "warning");
    notes.add(MapNoteFactory::instanceOf(NoteSetMaxVelocity::TYPE, "22"));

    cout << notes << endl;

    ASSERT_FALSE(notes.add(NoteSetMaxVelocity::TYPE, "33")) <<
        "The note was added regardless being a duplicate";

    ASSERT_TRUE(notes.has(NotePlaySound::TYPE)) <<
        "The sound note was not found";

    ASSERT_TRUE(notes.has(NoteSetMaxVelocity::TYPE)) <<
        "The max_velocity note was not found";

    shared_ptr<NoteSetMaxVelocity> maxVelocity = notes.get<NoteSetMaxVelocity>(NoteSetMaxVelocity::TYPE);

    ASSERT_FLOAT_EQ(22.0, maxVelocity->getMaxVelocity()) <<
        "The max velocity is not as expected";
}

TEST(Test_MapNotes, Equal)
{
    MapNotes notes1;
    notes1.add(NotePlaySound::TYPE, "warning");
    notes1.add(MapNoteFactory::instanceOf(NoteSetMaxVelocity::TYPE, "22"));

    MapNotes notes2;
    notes2.add(NotePlaySound::TYPE, "warning");
    notes2.add(MapNoteFactory::instanceOf(NoteSetMaxVelocity::TYPE, "22"));

    ASSERT_EQ(notes1, notes2) <<
        "The notes are expected to be equal";

    MapNotes notes3;
    notes3.add(NotePlaySound::TYPE, "warning");

    ASSERT_NE(notes1, notes3) <<
        "The notes are not expected to be equal";
}

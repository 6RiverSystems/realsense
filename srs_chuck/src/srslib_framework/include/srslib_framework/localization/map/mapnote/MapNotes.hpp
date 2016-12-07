/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
#include <sstream>
#include <unordered_set>
using namespace std;

#include <srslib_framework/localization/map/mapnote/MapNote2.hpp>

namespace srs {

class MapNotes : public unordered_set<MapNote2*, MapNote2, MapNote2>
{
public:
    MapNotes()
    {}

    MapNote2* find(MapNote2::MapNoteEnum noteType);

    virtual ~MapNotes()
    {
//        for (auto note : this)
//        {
//            if (note->isMutable())
//            {
//                delete note;
//            }
//        }
    }
};

} // namespace srs

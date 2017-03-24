/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <memory>
#include <string>
#include <sstream>
#include <unordered_map>
using namespace std;

#include <srslib_framework/localization/map/mapnote/MapNote.hpp>

namespace srs {

struct MapNoteFactory
{
    static MapNote::BaseMapNoteType instanceOf(const string& field, const string& value);
};

} // namespace srs

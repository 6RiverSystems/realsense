/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/localization/map/exception/MapStackException.hpp>

namespace srs {

class MissingTagException: public MapStackException
{
public:
    MissingTagException(const string& jsonFilename, const string& parameter) :
        MapStackException(jsonFilename,
            "YAML tag [" + parameter + "] not found or invalid")
    {}
};

} // namespace srs

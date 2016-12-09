/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/exception/SrsLogicErrorException.hpp>

namespace srs {

class YamlTagException: public SrsLogicErrorException
{
public:
    YamlTagException(const string& jsonFilename, const string& parameter) :
        SrsLogicErrorException("YAML tag not found or invalid [" +
            jsonFilename + ", " + parameter + "]")
    {}
};

} // namespace srs

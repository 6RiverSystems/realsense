/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <string>
using namespace std;

#include <yaml-cpp/yaml.h>

#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

namespace srs {

struct LogicalMapFactory
{
    static LogicalMap* fromJsonFile(string jsonFilename);
    static LogicalMap* fromMetadata(LogicalMetadata metadata);

private:
    static bool findId(YAML::Node node, string id, YAML::Node& result);

    static LogicalMap* nonTerminalMap(YAML::Node mapNode);

    static LogicalMap* synthesizeMap(YAML::Node documentNode);
};

} // namespace srs

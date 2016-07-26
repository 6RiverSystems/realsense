/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MAPUTILS_HPP_
#define MAPUTILS_HPP_

#include <string>
using namespace std;

#include <srslib_framework/localization/Map.hpp>

namespace srs {
namespace test {

struct MapUtils
{
    static Map* mapFactory(string mapFileName)
    {
        boost::filesystem::path filePath = boost::filesystem::canonical(mapFileName);

        Map* map = new Map();
        map->load(filePath.generic_string());

        return map;
    }
};

} // namespace test
} // namespace srs

#endif // MAPUTILS_HPP_

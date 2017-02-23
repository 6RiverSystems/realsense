/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

namespace srs {

struct MapStackMetadata
{
public:
    MapStackMetadata(double loadTime = 0, std::string mapStackFilename = "",
        std::string mapName = "", std::string mapVersion = "") :
            loadTime(loadTime),
            mapStackFilename(mapStackFilename),
            mapName(mapName),
            mapVersion(mapVersion)
    {}

    friend bool operator==(const MapStackMetadata& lhs, const MapStackMetadata& rhs)
    {
        return BasicMath::equal(lhs.loadTime, rhs.loadTime, 1e-10) &&
            lhs.mapStackFilename == rhs.mapStackFilename &&
            lhs.mapName == rhs.mapName &&
            lhs.mapVersion == rhs.mapVersion;
    }

public:
    double loadTime;

    std::string mapName;
    std::string mapStackFilename;
    std::string mapVersion;
};

} // namespace srs

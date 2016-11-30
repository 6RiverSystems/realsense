#include <srslib_framework/localization/map/MapStackFactory.hpp>

#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/exception/yaml/YamlTagException.hpp>
#include <srslib_framework/utils/Filesystem.hpp>

#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>

namespace srs {

const string MapStackFactory::TAG_LOGICAL = "logical";
const string MapStackFactory::TAG_LOGICAL_MAP = "map";

const string MapStackFactory::TAG_OCCUPANCY = "occupancy";
const string MapStackFactory::TAG_OCCUPANCY_RESOLUTION = "resolution";
const string MapStackFactory::TAG_OCCUPANCY_FREE_THRESHOLD = "free_thresh";
const string MapStackFactory::TAG_OCCUPANCY_OCCUPIED_THRESHOLD = "occupied_thresh";
const string MapStackFactory::TAG_OCCUPANCY_NEGATE = "negate";
const string MapStackFactory::TAG_OCCUPANCY_ORIGIN = "origin";
const string MapStackFactory::TAG_OCCUPANCY_IMAGE = "image";

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

MapStack* MapStackFactory::fromJsonFile(string jsonFilename, double loadingTime)
{
    YAML::Node mapStackDocument = YAML::LoadFile(jsonFilename);
    if (mapStackDocument.IsNull())
    {
        throw FailedToOpenFileException(jsonFilename);
    }

    string localDirectory = Filesystem::dirname(jsonFilename) + "/";

    LogicalMap* logical = analizeLogicalNode(
        localDirectory, jsonFilename, loadingTime, mapStackDocument);
    OccupancyMap* occupancy = analizeOccupancyNode(
        localDirectory, jsonFilename, loadingTime, mapStackDocument);

    return new MapStack(logical, occupancy, nullptr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* MapStackFactory::analizeLogicalNode(string localDirectory, string jsonFilename,
    double loadTime, YAML::Node& mapStackDocument)
{
    YAML::Node logicalDocument;
    try
    {
        logicalDocument = mapStackDocument[TAG_LOGICAL];
    }
    catch (YAML::InvalidScalar& e)
    {
        throw YamlTagException(jsonFilename, TAG_LOGICAL);
    }

    string logicalMapFilename;
    try
    {
        logicalMapFilename = logicalDocument[TAG_LOGICAL_MAP].as<string>();

        if (logicalMapFilename.size() == 0)
        {
            throw YamlTagException(jsonFilename, TAG_LOGICAL_MAP);
        }

        if (logicalMapFilename[0] != '/')
        {
            logicalMapFilename = localDirectory + logicalMapFilename;
        }

    }
    catch (YAML::InvalidScalar& e)
    {
        throw YamlTagException(jsonFilename, TAG_LOGICAL_MAP);
    }

    LogicalMapFactory logicalMapFactory;
    return logicalMapFactory.fromJsonFile(logicalMapFilename, loadTime);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* MapStackFactory::analizeOccupancyNode(string localDirectory, string jsonFilename,
    double loadTime, YAML::Node& mapStackDocument)
{
    YAML::Node occupancyDocument;
    try
    {
        occupancyDocument = mapStackDocument[TAG_OCCUPANCY];
    }
    catch (YAML::InvalidScalar& e)
    {
        throw YamlTagException(jsonFilename, TAG_OCCUPANCY);
    }

    OccupancyMetadata metadata;
    metadata.loadTime = loadTime;

    try
    {
        metadata.resolution = occupancyDocument[TAG_OCCUPANCY_RESOLUTION].as<double>();
    }
    catch (YAML::InvalidScalar& e)
    {
        throw YamlTagException(jsonFilename, TAG_OCCUPANCY_RESOLUTION);
    }

    try
    {
        metadata.thresholdFree = occupancyDocument[TAG_OCCUPANCY_FREE_THRESHOLD].as<double>();
    }
    catch (YAML::InvalidScalar& e)
    {
        throw YamlTagException(jsonFilename, TAG_OCCUPANCY_FREE_THRESHOLD);
    }

    try
    {
        metadata.thresholdOccupied = occupancyDocument[TAG_OCCUPANCY_OCCUPIED_THRESHOLD].as<double>();
    }
    catch (YAML::InvalidScalar& e)
    {
        throw YamlTagException(jsonFilename, TAG_OCCUPANCY_OCCUPIED_THRESHOLD);
    }

    try
    {
        metadata.negate = static_cast<bool>(occupancyDocument[TAG_OCCUPANCY_NEGATE].as<int>());
    }
    catch (YAML::InvalidScalar& e)
    {
        throw YamlTagException(jsonFilename, TAG_OCCUPANCY_NEGATE);
    }

    try {
        metadata.origin = Pose<>(
            occupancyDocument[TAG_OCCUPANCY_ORIGIN][0].as<double>(),
            occupancyDocument[TAG_OCCUPANCY_ORIGIN][1].as<double>(),
            occupancyDocument[TAG_OCCUPANCY_ORIGIN][2].as<double>());
    }
    catch (YAML::InvalidScalar& e)
    {
        throw YamlTagException(jsonFilename, TAG_OCCUPANCY_ORIGIN);
    }

    try
    {
        metadata.occupancyFilename = occupancyDocument[TAG_OCCUPANCY_IMAGE].as<string>();

        if (metadata.occupancyFilename.size() == 0)
        {
            throw;
        }

        if (metadata.occupancyFilename[0] != '/')
        {
            metadata.occupancyFilename = localDirectory + metadata.occupancyFilename;
        }

    }
    catch (YAML::InvalidScalar& e)
    {
        throw YamlTagException(jsonFilename, TAG_OCCUPANCY_IMAGE);
    }

    OccupancyMapFactory occupancyMapFactory;
    return occupancyMapFactory.fromMetadata(metadata);
}

} // namespace srs

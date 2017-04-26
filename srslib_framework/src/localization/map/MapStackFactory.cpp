#include <srslib_framework/localization/map/MapStackFactory.hpp>

#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/MapStackMetadata.hpp>
#include <srslib_framework/localization/map/exception/MissingTagException.hpp>
#include <srslib_framework/localization/map/exception/IncompatibleLanguageVersionException.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>
#include <srslib_framework/utils/Filesystem.hpp>

namespace srs {

const std::string MapStackFactory::LANGUAGE_VERSION = "1.1.0";

const std::string MapStackFactory::TAG_LANGUAGE_VERSION = "language_version";

const std::string MapStackFactory::TAG_METADATA_SECTION = "metadata";
const std::string MapStackFactory::TAG_METADATA_MAP_NAME = "map_name";
const std::string MapStackFactory::TAG_METADATA_MAP_VERSION = "map_version";

const string MapStackFactory::TAG_LOGICAL_SECTION = "logical";
const string MapStackFactory::TAG_LOGICAL_MAP = "map";

const string MapStackFactory::TAG_OCCUPANCY_SECTION = "occupancy";
const string MapStackFactory::TAG_OCCUPANCY_MAP = "map";
const string MapStackFactory::TAG_OCCUPANCY_RESOLUTION = "resolution";
const string MapStackFactory::TAG_OCCUPANCY_FREE_THRESHOLD = "free_thresh";
const string MapStackFactory::TAG_OCCUPANCY_OCCUPIED_THRESHOLD = "occupied_thresh";
const string MapStackFactory::TAG_OCCUPANCY_NEGATE = "negate";
const string MapStackFactory::TAG_OCCUPANCY_ORIGIN = "origin";

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

MapStack* MapStackFactory::fromJsonFile(string jsonFilename, double loadTime)
{
    YAML::Node mapStackDocument;
    try
    {
        mapStackDocument = YAML::LoadFile(jsonFilename);
    }
    catch (exception& e)
    {
        throw FailedToOpenFileException(jsonFilename);
    }

    Context context;
    context.localDirectory = Filesystem::dirname(jsonFilename) + "/";
    context.jsonFilename = jsonFilename;

    context.metadata.mapStackFilename = jsonFilename;
    context.metadata.loadTime = loadTime;

    checkLanguageVersion(context, mapStackDocument);
    readMetadata(context, mapStackDocument);

    LogicalMap* logical = analyzeLogicalNode(context, mapStackDocument);
    OccupancyMap* occupancy = analyzeOccupancyNode(context, mapStackDocument);

    return new MapStack(context.metadata, logical, occupancy, nullptr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* MapStackFactory::analyzeLogicalNode(Context& context, YAML::Node& mapStackDocument)
{
    YAML::Node logicalDocument;
    try
    {
        logicalDocument = mapStackDocument[TAG_LOGICAL_SECTION];
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_LOGICAL_SECTION);
    }

    string logicalMapFilename;
    try
    {
        logicalMapFilename = logicalDocument[TAG_LOGICAL_MAP].as<string>();

        if (logicalMapFilename.size() == 0)
        {
            throw MissingTagException(context.jsonFilename, TAG_LOGICAL_MAP);
        }

        if (logicalMapFilename[0] != '/')
        {
            logicalMapFilename = context.localDirectory + logicalMapFilename;
        }
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_LOGICAL_MAP);
    }

    LogicalMapFactory logicalMapFactory;
    return logicalMapFactory.fromJsonFile(logicalMapFilename);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* MapStackFactory::analyzeOccupancyNode(Context& context, YAML::Node& mapStackDocument)
{
    YAML::Node occupancyDocument;
    try
    {
        occupancyDocument = mapStackDocument[TAG_OCCUPANCY_SECTION];
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_OCCUPANCY_SECTION);
    }

    OccupancyMetadata metadata;

    try
    {
        metadata.resolution = occupancyDocument[TAG_OCCUPANCY_RESOLUTION].as<double>();
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_OCCUPANCY_RESOLUTION);
    }

    try
    {
        metadata.thresholdFree = occupancyDocument[TAG_OCCUPANCY_FREE_THRESHOLD].as<double>();
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_OCCUPANCY_FREE_THRESHOLD);
    }

    try
    {
        metadata.thresholdOccupied = occupancyDocument[TAG_OCCUPANCY_OCCUPIED_THRESHOLD].as<double>();
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_OCCUPANCY_OCCUPIED_THRESHOLD);
    }

    try
    {
        metadata.negate = static_cast<bool>(occupancyDocument[TAG_OCCUPANCY_NEGATE].as<int>());
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_OCCUPANCY_NEGATE);
    }

    try {
        metadata.origin = Pose<>(
            occupancyDocument[TAG_OCCUPANCY_ORIGIN][0].as<double>(),
            occupancyDocument[TAG_OCCUPANCY_ORIGIN][1].as<double>(),
            occupancyDocument[TAG_OCCUPANCY_ORIGIN][2].as<double>());
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_OCCUPANCY_ORIGIN);
    }

    try
    {
        metadata.occupancyFilename = occupancyDocument[TAG_OCCUPANCY_MAP].as<string>();

        if (metadata.occupancyFilename.size() == 0)
        {
            throw;
        }

        if (metadata.occupancyFilename[0] != '/')
        {
            metadata.occupancyFilename = context.localDirectory + metadata.occupancyFilename;
        }

    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_OCCUPANCY_MAP);
    }

    OccupancyMapFactory occupancyMapFactory;
    return occupancyMapFactory.fromMetadata(metadata);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapStackFactory::checkLanguageVersion(Context& context, YAML::Node& mapStackDocument)
{
    string languageVersion;
    try
    {
        languageVersion = mapStackDocument[TAG_LANGUAGE_VERSION].as<string>();

        if (languageVersion.size() == 0)
        {
            throw MissingTagException(context.jsonFilename, TAG_LANGUAGE_VERSION);
        }

        if (LANGUAGE_VERSION != languageVersion) {
            throw IncompatibleLanguageVersionException(context.jsonFilename,
                LANGUAGE_VERSION, languageVersion);
        }
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_LANGUAGE_VERSION);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapStackFactory::readMetadata(Context& context, YAML::Node& mapStackDocument)
{
    YAML::Node metadataDocument;
    try
    {
        metadataDocument = mapStackDocument[TAG_METADATA_SECTION];
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_METADATA_SECTION);
    }

    try
    {
        context.metadata.mapName = metadataDocument[TAG_METADATA_MAP_NAME].as<string>();
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_METADATA_MAP_NAME);
    }

    try
    {
        context.metadata.mapVersion = metadataDocument[TAG_METADATA_MAP_VERSION].as<string>();
    }
    catch (YAML::InvalidScalar& e)
    {
        throw MissingTagException(context.jsonFilename, TAG_METADATA_MAP_VERSION);
    }

    context.metadata.mapStackFilename = context.jsonFilename;
}

} // namespace srs

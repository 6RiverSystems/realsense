#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>

#include <algorithm>
using namespace std;

#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/logical/exception/FeatureNotFoundException.hpp>
#include <srslib_framework/localization/map/logical/exception/FeaturesNotFoundException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedFeatureException.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromJsonFile(string jsonFilename)
{
    YAML::Node jsonDocument = YAML::LoadFile(jsonFilename);
    if (jsonDocument.IsNull())
    {
        throw FailedToOpenFileException(jsonFilename);
    }

    LogicalMetadata metadata;
    metadata.logicalFilename = jsonFilename;

    LogicalMap* logicalMap = nonTerminalEntry(metadata, jsonDocument);
    logicalMap->setLogicalFilename(jsonFilename);

    return logicalMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromString(string geoJson)
{
    LogicalMetadata metadata;
    YAML::Node jsonDocument = YAML::Load(geoJson);

    return nonTerminalEntry(metadata, jsonDocument);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addRectangleCost(LogicalMap* logicalMap,
    Pose<> origin, double widthMm, double heightMm,
    unsigned int cost)
{
    double newWidthMm = widthMm;
    double c = origin.x;
    if (c < 0)
    {
        newWidthMm += c;
        newWidthMm = newWidthMm > 0 ? newWidthMm : 0;
        c = 0;
    }

    double newHeightMm = heightMm;
    double r = origin.y;
    if (r < 0)
    {
        newHeightMm += r;
        newHeightMm = newHeightMm > 0 ? newHeightMm : 0;
        r = 0;
    }

    unsigned int x0;
    unsigned int y0;
    logicalMap->transformMm2Cells(c, r, x0, y0);

    unsigned int widthCells;
    logicalMap->transformMm2Cells(newWidthMm, widthCells);

    unsigned int heightCells;
    logicalMap->transformMm2Cells(newHeightMm, heightCells);

    for (unsigned int r = y0; r < y0 + heightCells; ++r)
    {
        for (unsigned int c = x0; c < x0 + widthCells; ++c)
        {
            logicalMap->addCost(c, r, cost);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addStaticObstacle(LogicalMap* logicalMap,
    Pose<> origin, double widthMm, double heightMm,
    double sizeEnvelope, unsigned int costEnvelope)
{
    // First add the envelope, if specified
    if (sizeEnvelope > 0.0 && costEnvelope > 0)
    {
        addRectangleCost(logicalMap,
            PoseMath::add(origin, Pose<>(-sizeEnvelope, -sizeEnvelope)),
            widthMm + 2 * sizeEnvelope, heightMm + 2 * sizeEnvelope,
            costEnvelope);
    }

    // Add the static obstacle
    addRectangleCost(logicalMap, origin,
        widthMm, heightMm, numeric_limits<unsigned int>::max());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool LogicalMapFactory::findCollection(YAML::Node node, YAML::Node& result)
{
    if (node.IsSequence())
    {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        {
            YAML::Node element = *it;

            string value = element["type"].as<string>();
            if (value == "FeatureCollection")
            {
                result = element["features"];
                return true;
            }
        }
    }
    else
    {
        if (node["type"])
        {
            string value = node["type"].as<string>();
            if (value == "FeatureCollection")
            {
                result = node["features"];
                return true;
            }
        }
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool LogicalMapFactory::findIdInCollection(YAML::Node node, string id, YAML::Node& result)
{
    if (node.IsSequence())
    {
        for (YAML::const_iterator it = node.begin(); it != node.end(); ++it)
        {
            YAML::Node element = *it;
            if (element["id"])
            {
                string value = element["id"].as<string>();
                if (value == id)
                {
                    result = element;
                    return true;
                }
            }
        }
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalBoundary(YAML::Node node, LogicalMap* map)
{
    YAML::Node properties = node["properties"];

    double envelopeSize = 0.0;
    if (properties["envelope_size"])
    {
        envelopeSize = properties["envelope_size"].as<double>();
    }

    unsigned int envelopeCost = 0;
    if (properties["envelope_cost"])
    {
        envelopeCost = properties["envelope_cost"].as<unsigned int>();
    }

    double widthM = map->getWidthMm();
    double heightM = map->getHeightMm();
    double resolution = map->getResolution();

    YAML::Node coordinates = node["geometry"]["coordinates"];
    Pose<> c0 = coordinates[0].as<Pose<double>>();
    Pose<> c2 = coordinates[2].as<Pose<double>>();

    // Add the bottom border
    addStaticObstacle(map, Pose<>::ZERO, widthM, c0.y,
        envelopeSize, envelopeCost);

    // Add the left border
    addStaticObstacle(map, Pose<>(0, c0.y), c0.x, heightM - c0.y,
        envelopeSize, envelopeCost);

    // Add the top border
    addStaticObstacle(map, Pose<>(c0.x, c2.y), widthM - c0.x, heightM - c2.y,
        envelopeSize, envelopeCost);

    // Add the right border
    addStaticObstacle(map, Pose<>(c2.x, c0.y), widthM - c2.x, c2.y - c0.y,
        envelopeSize, envelopeCost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::nonTerminalEntry(LogicalMetadata& metadata, YAML::Node node)
{
    LogicalMap* logicalMap = nullptr;

    YAML::Node features;
    if (findCollection(node, features))
    {
        YAML::Node mapNode;
        if (findIdInCollection(features, "map", mapNode))
        {
            logicalMap = nonTerminalMap(mapNode);
        }
        else
        {
            throw FeatureNotFoundException(metadata.logicalFilename, "map");
        }

        YAML::Node boundaryNode;
        if (findIdInCollection(features, "boundary", boundaryNode))
        {
            nonTerminalBoundary(boundaryNode, logicalMap);
        }

        YAML::Node obstaclesNode;
        if (findCollection(features, obstaclesNode))
        {
            nonTerminalObstacles(obstaclesNode, logicalMap);
        }
    }
    else
    {
        throw FeaturesNotFoundException(metadata.logicalFilename);
    }

    return logicalMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::nonTerminalMap(YAML::Node node)
{
    YAML::Node properties = node["properties"];

    Pose<> origin = properties["origin"].as<Pose<>>();
    double resolution = properties["resolution"].as<double>();
    double width = properties["width"].as<double>();
    double height = properties["height"].as<double>();

    LogicalMap* logicalMap = new LogicalMap(width, height, resolution);
    logicalMap->setOrigin(origin);

    return logicalMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalObstacle(YAML::Node node, LogicalMap* map)
{
    YAML::Node properties = node["properties"];

    double envelopeSize = 0.0;
    if (properties["envelope_size"])
    {
        envelopeSize = properties["envelope_size"].as<double>();
    }

    unsigned int envelopeCost = 0;
    if (properties["envelope_cost"])
    {
        envelopeCost = properties["envelope_cost"].as<unsigned int>();
    }

    YAML::Node coordinates = node["geometry"]["coordinates"];

    Pose<> c0 = coordinates[0].as<Pose<double>>();
    Pose<> c2 = coordinates[2].as<Pose<double>>();

    // Add the static obstacle
    addStaticObstacle(map, c0, abs(c2.x - c0.x), abs(c2.y - c0.y),
        envelopeSize, envelopeCost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalObstacles(YAML::Node node, LogicalMap* map)
{
    // Go through all the obstacles in the collection
    for (auto obstacle : node)
    {
        if (obstacle["id"].as<string>() == "obstacle")
        {
            nonTerminalObstacle(obstacle, map);
        }
        else
        {
            throw UnexpectedFeatureException(map->getMetadata().logicalFilename,
                obstacle["id"].as<string>());
        }
    }
}

} // namespace srs

////////////////////////////////////////////////////////////////////////////////////////////////////
namespace YAML {

////////////////////////////////////////////////////////////////////////////////////////////////////
template<>
struct convert<srs::Pose<double>>
{
  static bool decode(const Node& node, srs::Pose<double>& rhs)
  {
      // Check if it is a simple or full pose (without theta or with theta)
      if (node.size() == 2 || node.size() == 3)
      {
          rhs.x = node[0].as<double>();
          rhs.y = node[1].as<double>();

          // If theta is expected
          if (node.size() == 3)
          {
              rhs.theta = node[2].as<double>();
          }

          return true;
      }

      return false;
  }
};

} // namespace YAML

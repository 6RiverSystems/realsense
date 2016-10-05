#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>

#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/logical/exception/FeatureNotFoundException.hpp>
#include <srslib_framework/localization/map/logical/exception/FeaturesNotFoundException.hpp>

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

    LogicalMap* logicalMap = LogicalMapFactory::synthesizeMap(metadata, jsonDocument);
    logicalMap->setLogicalFilename(jsonFilename);

    return logicalMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromString(string geoJson)
{
    LogicalMetadata metadata;
    YAML::Node jsonDocument = YAML::Load(geoJson);

    return LogicalMapFactory::synthesizeMap(metadata, jsonDocument);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addRectangleCost(LogicalMap* logicalMap,
    Pose<> origin, double widthM, double heightM,
    unsigned int cost)
{
    unsigned int x0;
    unsigned int y0;
    logicalMap->transformM2Cells(origin, x0, y0);

    unsigned int widthCells;
    logicalMap->transformM2Cells(widthM, widthCells);

    unsigned int heightCells;
    logicalMap->transformM2Cells(heightM, heightCells);

    for (unsigned int r = y0; r < y0 + heightCells; ++r)
    {
        for (unsigned int c = x0; c < x0 + widthCells; ++c)
        {
            logicalMap->setCost(c, r, cost);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addStaticObstacle(LogicalMap* logicalMap,
    Pose<> origin, double widthM, double heightM,
    double sizeEnvelope, unsigned int costEnvelope)
{
    // First add the envelope, if specified
    if (sizeEnvelope > 0.0 && costEnvelope > 0)
    {
        LogicalMapFactory::addRectangleCost(logicalMap,
            PoseMath::add(origin, Pose<>(-sizeEnvelope, -sizeEnvelope)),
            widthM + sizeEnvelope, heightM + sizeEnvelope,
            costEnvelope);
    }

    // Add the static obstacle
    LogicalMapFactory::addRectangleCost(logicalMap, origin,
        widthM, heightM, numeric_limits<unsigned int>::max());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool LogicalMapFactory::findId(YAML::Node node, string id, YAML::Node& result)
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
void LogicalMapFactory::nonTerminalBoundary(YAML::Node node, LogicalMap* logicalMap)
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

    double widthM = logicalMap->getWidthMeters();
    double heightM = logicalMap->getHeightMeters();
    double resolution = logicalMap->getResolution();

    YAML::Node coordinates = node["geometry"]["coordinates"];
    Pose<> c0 = coordinates[0].as<Pose<double>>();
    Pose<> c2 = coordinates[2].as<Pose<double>>();

    // Add the bottom border
    LogicalMapFactory::addStaticObstacle(logicalMap,
        Pose<>::ZERO, widthM, c0.y,
        envelopeSize, envelopeCost);

    // Add the left border
    LogicalMapFactory::addStaticObstacle(logicalMap,
        Pose<>(0, c0.y), c0.x, heightM - c0.y,
        envelopeSize, envelopeCost);

    // Add the top border
    LogicalMapFactory::addStaticObstacle(logicalMap,
        Pose<>(c0.x, c2.y), widthM - c0.x, heightM - c2.y,
        envelopeSize, envelopeCost);

    // Add the right border
    LogicalMapFactory::addStaticObstacle(logicalMap,
        Pose<>(c2.x, c0.y), widthM - c2.x, c2.y - c0.y,
        envelopeSize, envelopeCost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::nonTerminalMap(YAML::Node node)
{
    YAML::Node properties = node["properties"];
    double resolution = properties["resolution"].as<double>();
    Pose<> origin = properties["origin"].as<Pose<>>();

    YAML::Node coordinates = node["geometry"]["coordinates"];
    Pose<> c0 = coordinates[0].as<Pose<double>>();
    Pose<> c1 = coordinates[1].as<Pose<double>>();
    Pose<> c2 = coordinates[2].as<Pose<double>>();
    Pose<> c3 = coordinates[3].as<Pose<double>>();

    LogicalMap* logicalMap = new LogicalMap(abs(c1.x - c0.x), abs(c2.y - c1.y), resolution);
    logicalMap->setOrigin(origin);

    return logicalMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::synthesizeMap(LogicalMetadata& metadata, YAML::Node documentNode)
{
    LogicalMap* logicalMap = nullptr;

    YAML::Node features = documentNode["features"];
    if (features)
    {
        YAML::Node mapNode;
        if (LogicalMapFactory::findId(features, "map", mapNode))
        {
            logicalMap = nonTerminalMap(mapNode);
        }
        else
        {
            throw FeatureNotFoundException(metadata.logicalFilename, "map");
        }

        YAML::Node boundaryNode;
        if (LogicalMapFactory::findId(features, "boundary", boundaryNode))
        {
            nonTerminalBoundary(boundaryNode, logicalMap);
        }
    }
    else
    {
        throw FeaturesNotFoundException(metadata.logicalFilename);
    }

    return logicalMap;
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

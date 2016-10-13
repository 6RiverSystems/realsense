#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>

#include <algorithm>
using namespace std;

#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/logical/exception/FeatureNotFoundException.hpp>
#include <srslib_framework/localization/map/logical/exception/FeaturesNotFoundException.hpp>
#include <srslib_framework/localization/map/logical/exception/GeoJsonTypeUnsupportedException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedFeatureException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedNumberOfPointsException.hpp>

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
    int cost)
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
            logicalMap->maxCost(c, r, cost);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addStaticObstacle(LogicalMap* logicalMap,
    Pose<> origin, double widthMm, double heightMm,
    double sizeEnvelopeMm, int costEnvelope)
{
    // First add the envelope, if specified
    if (sizeEnvelopeMm > 0.0 && costEnvelope > 0)
    {
        addRectangleCost(logicalMap,
            PoseMath::add(origin, Pose<>(-sizeEnvelopeMm, -sizeEnvelopeMm)),
            widthMm + 2 * sizeEnvelopeMm, heightMm + 2 * sizeEnvelopeMm,
            costEnvelope);
    }

    // Add the static obstacle
    addRectangleCost(logicalMap, origin, widthMm, heightMm, Grid2d::COST_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addWeight(LogicalMap* logicalMap,
    Pose<> from, Pose<> to,
    int north, int east, int south, int west)
{
    unsigned int xi;
    unsigned int yi;
    logicalMap->transformMm2Cells(from, xi, yi);

    unsigned int xf;
    unsigned int yf;
    logicalMap->transformMm2Cells(to, xf, yf);

    int deltaX = BasicMath::sgn<int>(xf - xi);
    int deltaY = BasicMath::sgn<int>(yf - yi);

    unsigned int c;
    unsigned int r = yi;
    do
    {
        c = xi;
        do
        {
            logicalMap->setWeights(c, r, north, east, south, west);
            c += deltaX;
        } while (c != (xf + deltaX));

        r += deltaY;
    } while (r != (yf + deltaY));
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
int LogicalMapFactory::nonTerminalCostValue(YAML::Node node, LogicalMap* map)
{
    if (node)
    {
        int cost = Grid2d::COST_MAX;

        string costString = node.as<string>();
        if (costString != "max")
        {
            cost = node.as<int>();
        }

        return cost;
    }

    return Grid2d::COST_MIN;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalEntities(YAML::Node node, LogicalMap* map)
{
    // Go through all the obstacles in the collection
    for (auto entity : node)
    {
        string entityId = entity["id"].as<string>();
        if (entityId == "obstacle")
        {
            nonTerminalStatementObstacle(entity, map);
        }
        else if (entityId == "weight")
        {
            nonTerminalStatementWeight(entity, map);
        }
        else
        {
            throw UnexpectedFeatureException(map->getMetadata(), entityId);
        }
    }
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
            throw FeatureNotFoundException(metadata, "map");
        }

        YAML::Node boundaryNode;
        if (findIdInCollection(features, "boundary", boundaryNode))
        {
            nonTerminalStatementBoundary(boundaryNode, logicalMap);
        }

        YAML::Node entitiesNode;
        if (findCollection(features, entitiesNode))
        {
            nonTerminalEntities(entitiesNode, logicalMap);
        }
    }
    else
    {
        throw FeaturesNotFoundException(metadata);
    }

    return logicalMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Pose<>> LogicalMapFactory::nonTerminalGeometry(YAML::Node node, LogicalMap* map)
{
    vector<Pose<>> coordinates;

    YAML::Node typeNode = node["type"];
    if (typeNode)
    {
        YAML::Node coordinatesNode = node["coordinates"];

        string typeString = typeNode.as<string>();
        if (typeString == "Point")
        {
            coordinates.push_back(coordinatesNode.as<Pose<>>());
        }
        else if (typeString == "MultiPoint")
        {
            for (auto coordinate : coordinatesNode)
            {
                coordinates.push_back(coordinate.as<Pose<>>());
            }
        }
        else
        {
            throw GeoJsonTypeUnsupportedException(map->getMetadata(), typeString);
        }
    }

    return coordinates;
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
void LogicalMapFactory::nonTerminalStatementBoundary(YAML::Node node, LogicalMap* map)
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

    vector<Pose<>> coordinates = nonTerminalGeometry(node["geometry"], map);
    if (coordinates.size() == 2)
    {
        Pose<> bl = coordinates[0];
        Pose<> tr = coordinates[1];

        // Add the bottom border
        addStaticObstacle(map, Pose<>::ZERO, widthM, bl.y,
            envelopeSize, envelopeCost);

        // Add the left border
        addStaticObstacle(map, Pose<>(0, bl.y), bl.x, heightM - bl.y,
            envelopeSize, envelopeCost);

        // Add the top border
        addStaticObstacle(map, Pose<>(bl.x, tr.y), widthM - bl.x, heightM - tr.y,
            envelopeSize, envelopeCost);

        // Add the right border
        addStaticObstacle(map, Pose<>(tr.x, bl.y), widthM - tr.x, tr.y - bl.y,
            envelopeSize, envelopeCost);
    }
    else
    {
        throw UnexpectedNumberOfPointsException(map->getMetadata(), 2, coordinates.size());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalStatementObstacle(YAML::Node node, LogicalMap* map)
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

    vector<Pose<>> coordinates = nonTerminalGeometry(node["geometry"], map);
    if (coordinates.size() == 2)
    {
        Pose<> bl = coordinates[0];
        Pose<> tr = coordinates[1];

        // Add the static obstacle
        addStaticObstacle(map, bl, abs(tr.x - bl.x), abs(tr.y - bl.y),
            envelopeSize, envelopeCost);
    }
    else
    {
        throw UnexpectedNumberOfPointsException(map->getMetadata(), 2, coordinates.size());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalStatementWeight(YAML::Node node, LogicalMap* map)
{
    YAML::Node properties = node["properties"];

    int northCost = nonTerminalCostValue(properties["north"], map);
    int eastCost = nonTerminalCostValue(properties["east"], map);
    int southCost = nonTerminalCostValue(properties["south"], map);
    int westCost = nonTerminalCostValue(properties["west"], map);

    vector<Pose<>> coordinates = nonTerminalGeometry(node["geometry"], map);
    if (coordinates.size() == 2)
    {
        Pose<> from = coordinates[0];
        Pose<> to = coordinates[1];
        addWeight(map, from, to, northCost, eastCost, southCost, westCost);
    }
    else if (coordinates.size() == 1)
    {
        Pose<> from = coordinates[0];
        addWeight(map, from, from, northCost, eastCost, southCost, westCost);
    }
    else
    {
        throw UnexpectedNumberOfPointsException(map->getMetadata(), 1, 2, coordinates.size());
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

#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>

#include <algorithm>
using namespace std;

#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/exception/yaml/FailedToParseJsonException.hpp>
#include <srslib_framework/localization/map/logical/exception/CostExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/DoubleExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/FeatureNotFoundException.hpp>
#include <srslib_framework/localization/map/logical/exception/FeaturesNotFoundException.hpp>
#include <srslib_framework/localization/map/logical/exception/GeoJsonTypeUnsupportedException.hpp>
#include <srslib_framework/localization/map/logical/exception/InvalidCostValueException.hpp>
#include <srslib_framework/localization/map/logical/exception/PoseExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/PropertyNotFoundException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedFeatureException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedNumberOfPointsException.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods
const string LogicalMapFactory::KEYWORD_BOUNDARY = "boundary";

const string LogicalMapFactory::KEYWORD_COORDINATES = "coordinates";
const string LogicalMapFactory::KEYWORD_COST_AREA = "cost_area";

const string LogicalMapFactory::KEYWORD_FEATURE_COLLECTION = "FeatureCollection";
const string LogicalMapFactory::KEYWORD_FEATURES = "features";

const string LogicalMapFactory::KEYWORD_GEOMETRY = "geometry";

const string LogicalMapFactory::KEYWORD_ID = "id";

const string LogicalMapFactory::KEYWORD_MAP = "map";
const string LogicalMapFactory::KEYWORD_MAX = "max";

const string LogicalMapFactory::KEYWORD_OBSTACLE = "obstacle";

const string LogicalMapFactory::KEYWORD_PROPERTIES = "properties";
const string LogicalMapFactory::KEYWORD_PROPERTY_COST_AREA_COST = "cost";
const string LogicalMapFactory::KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_COST = "envelope_cost";
const string LogicalMapFactory::KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_SIZE = "envelope_size";
const string LogicalMapFactory::KEYWORD_PROPERTY_MAP_HEIGHT = "height";
const string LogicalMapFactory::KEYWORD_PROPERTY_MAP_ORIGIN = "origin";
const string LogicalMapFactory::KEYWORD_PROPERTY_MAP_RESOLUTION = "resolution";
const string LogicalMapFactory::KEYWORD_PROPERTY_MAP_WIDTH = "width";
const string LogicalMapFactory::KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_COST = "envelope_cost";
const string LogicalMapFactory::KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_SIZE = "envelope_size";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHT_AREA_EAST = "east";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHT_AREA_NORTH = "north";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHT_AREA_SOUTH = "south";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHT_AREA_WEST = "west";

const string LogicalMapFactory::KEYWORD_TYPE = "type";
const string LogicalMapFactory::KEYWORD_TYPE_POINT = "Point";
const string LogicalMapFactory::KEYWORD_TYPE_MULTIPOINT = "MultiPoint";

const string LogicalMapFactory::KEYWORD_WEIGHT_AREA = "weight_area";

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromGrid2d(Grid2d* grid, double resolution)
{
    map_ = new LogicalMap(grid, resolution);
    metadata_ = map_->getMetadata();

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromJsonFile(string jsonFilename, double loadTime)
{
    YAML::Node jsonDocument = YAML::LoadFile(jsonFilename);
    if (jsonDocument.IsNull())
    {
        throw FailedToOpenFileException(jsonFilename);
    }

    map_ = nullptr;

    metadata_ = LogicalMetadata();
    metadata_.loadTime = loadTime;
    metadata_.logicalFilename = jsonFilename;

    nonTerminalEntry(jsonDocument);

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromString(string geoJson, double loadTime)
{
    YAML::Node jsonDocument = YAML::Load(geoJson);
    if (jsonDocument.IsNull())
    {
        throw FailedToParseJsonException(geoJson);
    }

    map_ = nullptr;

    metadata_ = LogicalMetadata();
    metadata_.loadTime = loadTime;

    nonTerminalEntry(jsonDocument);

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addRectangleCost(Pose<> origin, double widthM, double heightM,
    Grid2d::BaseType cost)
{
    double newWidthM = widthM;
    double c = origin.x;
    if (c < 0)
    {
        newWidthM += c;
        newWidthM = newWidthM > 0 ? newWidthM : 0;
        c = 0;
    }

    double newHeightM = heightM;
    double r = origin.y;
    if (r < 0)
    {
        newHeightM += r;
        newHeightM = newHeightM > 0 ? newHeightM : 0;
        r = 0;
    }

    unsigned int x0;
    unsigned int y0;
    map_->transformM2Cells(c, r, x0, y0);

    unsigned int widthCells;
    map_->transformM2Cells(newWidthM, widthCells);
    widthCells = max<unsigned int>(1, widthCells);

    unsigned int heightCells;
    map_->transformM2Cells(newHeightM, heightCells);
    heightCells = max<unsigned int>(1, heightCells);

    for (unsigned int r = y0; r < y0 + heightCells; ++r)
    {
        for (unsigned int c = x0; c < x0 + widthCells; ++c)
        {
            if (map_->isWithinBounds(c, r))
            {
                map_->maxCost(c, r, cost);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addStaticObstacle(Pose<> origin, double widthM, double heightM,
    double sizeEnvelopeM, Grid2d::BaseType costEnvelope)
{
    // First add the envelope, if specified
    if (sizeEnvelopeM > 0.0 && costEnvelope > 0)
    {
        addRectangleCost(PoseMath::add(origin, Pose<>(-sizeEnvelopeM, -sizeEnvelopeM)),
            widthM + 2 * sizeEnvelopeM, heightM + 2 * sizeEnvelopeM,
            costEnvelope);
    }

    // Add the static obstacle
    addRectangleCost(origin, widthM, heightM, Grid2d::PAYLOAD_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addWeight(Pose<> from, Pose<> to,
    Grid2d::BaseType north,
    Grid2d::BaseType east,
    Grid2d::BaseType south,
    Grid2d::BaseType west)
{
    unsigned int xi;
    unsigned int yi;
    map_->transformM2Cells(from, xi, yi);

    unsigned int xf;
    unsigned int yf;
    map_->transformM2Cells(to, xf, yf);

    int deltaX = BasicMath::sgn<int>(xf - xi);
    int deltaY = BasicMath::sgn<int>(yf - yi);

    unsigned int c;
    unsigned int r = yi;
    do
    {
        c = xi;
        do
        {
            map_->setWeights(c, r, north, east, south, west);
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

            string value = element[KEYWORD_TYPE].as<string>();
            if (value == KEYWORD_FEATURE_COLLECTION)
            {
                result = element[KEYWORD_FEATURES];
                return !result.IsNull();
            }
        }
    }
    else
    {
        if (node[KEYWORD_TYPE])
        {
            string value = node[KEYWORD_TYPE].as<string>();
            if (value == KEYWORD_FEATURE_COLLECTION)
            {
                result = node[KEYWORD_FEATURES];
                return !result.IsNull();
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
            if (element[KEYWORD_ID])
            {
                string value = element[KEYWORD_ID].as<string>();
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
void LogicalMapFactory::nonTerminalEntities(YAML::Node node)
{
    // Go through all the obstacles in the collection
    for (auto entity : node)
    {
        string entityId = entity[KEYWORD_ID].as<string>();
        if (entityId == KEYWORD_OBSTACLE)
        {
            nonTerminalStatementObstacle(entity);
        }
        else if (entityId == KEYWORD_COST_AREA)
        {
            nonTerminalStatementCostArea(entity);
        }
        else if (entityId == KEYWORD_WEIGHT_AREA)
        {
            nonTerminalStatementWeightArea(entity);
        }
        else
        {
            throw UnexpectedFeatureException(metadata_, entityId);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalEntry(YAML::Node node)
{
    YAML::Node features;
    if (findCollection(node, features))
    {
        YAML::Node mapNode;
        if (findIdInCollection(features, KEYWORD_MAP, mapNode))
        {
            nonTerminalMap(mapNode);
        }
        else
        {
            throw FeatureNotFoundException(metadata_, KEYWORD_MAP);
        }

        YAML::Node boundaryNode;
        if (findIdInCollection(features, KEYWORD_BOUNDARY, boundaryNode))
        {
            nonTerminalStatementBoundary(boundaryNode);
        }

        YAML::Node entitiesNode;
        if (findCollection(features, entitiesNode))
        {
            nonTerminalEntities(entitiesNode);
        }
    }
    else
    {
        throw FeaturesNotFoundException(metadata_);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Pose<>> LogicalMapFactory::nonTerminalGeometry(YAML::Node node,
    int minNumber, int maxNumber)
{
    vector<Pose<>> coordinates;

    YAML::Node typeNode = node[KEYWORD_TYPE];
    if (typeNode)
    {
        YAML::Node coordinatesNode = node[KEYWORD_COORDINATES];

        string typeString = typeNode.as<string>();
        if (typeString == KEYWORD_TYPE_POINT)
        {
            coordinates.push_back(nonTerminalTypePoint(coordinatesNode, true));
        }
        else if (typeString == KEYWORD_TYPE_MULTIPOINT)
        {
            for (auto coordinate : coordinatesNode)
            {
                coordinates.push_back(nonTerminalTypePoint(coordinate, true));
            }
        }
        else
        {
            throw GeoJsonTypeUnsupportedException(metadata_, typeString);
        }
    }

    if (coordinates.size() < minNumber || coordinates.size() > maxNumber)
    {
        throw UnexpectedNumberOfPointsException(metadata_,
            minNumber, maxNumber, coordinates.size());
    }

    return coordinates;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalMap(YAML::Node node)
{
    YAML::Node properties = node[KEYWORD_PROPERTIES];

    metadata_.origin = nonTerminalTypePoint(properties[KEYWORD_PROPERTY_MAP_ORIGIN], false);
    metadata_.resolution = nonTerminalTypeDouble(properties[KEYWORD_PROPERTY_MAP_RESOLUTION], false);
    metadata_.widthM = nonTerminalTypeDouble(properties[KEYWORD_PROPERTY_MAP_WIDTH], false);
    metadata_.heightM = nonTerminalTypeDouble(properties[KEYWORD_PROPERTY_MAP_HEIGHT], false);

    map_ = new LogicalMap(metadata_);
    metadata_ = map_->getMetadata();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalStatementBoundary(YAML::Node node)
{
    YAML::Node properties = node[KEYWORD_PROPERTIES];

    double envelopeSize = nonTerminalTypeDouble(properties[KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_SIZE], false);
    Grid2d::BaseType envelopeCost = nonTerminalTypeCost(properties[KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_COST], false);

    double widthM = map_->getWidthM();
    double heightM = map_->getHeightM();
    double resolution = map_->getResolution();

    vector<Pose<>> coordinates = nonTerminalGeometry(node[KEYWORD_GEOMETRY], 2, 2);

    Pose<> bl = coordinates[0];
    Pose<> tr = coordinates[1];

    // Add the bottom border
    addStaticObstacle(Pose<>::ZERO, widthM, bl.y, envelopeSize, envelopeCost);

    // Add the left border
    addStaticObstacle(Pose<>(0, bl.y), bl.x, heightM - bl.y, envelopeSize, envelopeCost);

    // Add the top border
    addStaticObstacle(Pose<>(bl.x, tr.y), widthM - bl.x, heightM - tr.y, envelopeSize, envelopeCost);

    // Add the right border
    addStaticObstacle(Pose<>(tr.x, bl.y), widthM - tr.x, tr.y - bl.y, envelopeSize, envelopeCost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalStatementCostArea(YAML::Node node)
{
    YAML::Node properties = node[KEYWORD_PROPERTIES];

    Grid2d::BaseType cost = nonTerminalTypeCost(properties[KEYWORD_PROPERTY_COST_AREA_COST], true);

    vector<Pose<>> coordinates = nonTerminalGeometry(node[KEYWORD_GEOMETRY], 2, 2);

    Pose<> bl = coordinates[0];
    Pose<> tr = coordinates[1];

    // Add the static obstacle
    addRectangleCost(bl, abs(tr.x - bl.x), abs(tr.y - bl.y), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalStatementObstacle(YAML::Node node)
{
    YAML::Node properties = node[KEYWORD_PROPERTIES];

    double envelopeSize = nonTerminalTypeDouble(properties[KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_SIZE], false);
    Grid2d::BaseType envelopeCost = nonTerminalTypeCost(properties[KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_COST], false);

    vector<Pose<>> coordinates = nonTerminalGeometry(node[KEYWORD_GEOMETRY], 2, 2);

    Pose<> bl = coordinates[0];
    Pose<> tr = coordinates[1];

    // Add the static obstacle
    addStaticObstacle(bl, abs(tr.x - bl.x), abs(tr.y - bl.y), envelopeSize, envelopeCost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::nonTerminalStatementWeightArea(YAML::Node node)
{
    YAML::Node properties = node[KEYWORD_PROPERTIES];

    Grid2d::BaseType northCost = nonTerminalTypeCost(properties[KEYWORD_PROPERTY_WEIGHT_AREA_NORTH], false);
    Grid2d::BaseType eastCost = nonTerminalTypeCost(properties[KEYWORD_PROPERTY_WEIGHT_AREA_EAST], false);
    Grid2d::BaseType southCost = nonTerminalTypeCost(properties[KEYWORD_PROPERTY_WEIGHT_AREA_SOUTH], false);
    Grid2d::BaseType westCost = nonTerminalTypeCost(properties[KEYWORD_PROPERTY_WEIGHT_AREA_WEST], false);

    vector<Pose<>> coordinates = nonTerminalGeometry(node[KEYWORD_GEOMETRY], 1, 2);
    if (coordinates.size() == 2)
    {
        Pose<> from = coordinates[0];
        Pose<> to = coordinates[1];
        addWeight(from, to, northCost, eastCost, southCost, westCost);
    }
    else if (coordinates.size() == 1)
    {
        Pose<> from = coordinates[0];
        addWeight(from, from, northCost, eastCost, southCost, westCost);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2d::BaseType LogicalMapFactory::nonTerminalTypeCost(YAML::Node node, bool required)
{
    int value = Grid2d::PAYLOAD_MIN;

    if (node)
    {
        string cost = node.as<string>();

        try
        {
            value = cost != KEYWORD_MAX ? node.as<int>() : Grid2d::PAYLOAD_MAX;
        }
        catch (exception& e)
        {
            throw CostExpectedException(metadata_);
        }

        if (value < Grid2d::PAYLOAD_MIN || value > Grid2d::PAYLOAD_MAX)
        {
            throw InvalidCostValueException(metadata_, value,
                Grid2d::PAYLOAD_MIN, Grid2d::PAYLOAD_MAX);
        }
    }
    else if (required)
    {
        throw CostExpectedException(metadata_);
    }

    return static_cast<Grid2d::BaseType>(value);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
double LogicalMapFactory::nonTerminalTypeDouble(YAML::Node node, bool required)
{
    double value = 0.0;

    if (node)
    {
        try
        {
            value = node.as<double>();
        }
        catch (exception& e)
        {
            throw DoubleExpectedException(metadata_);
        }
    }
    else if (required)
    {
        throw DoubleExpectedException(metadata_);
    }

    return value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Pose<> LogicalMapFactory::nonTerminalTypePoint(YAML::Node node, bool required)
{
    Pose<> value = Pose<>::ZERO;

    if (node)
    {
        try
        {
            value = node.as<Pose<>>();
        }
        catch (exception& e)
        {
            throw PoseExpectedException(metadata_);
        }
    }
    else if (required)
    {
        throw PoseExpectedException(metadata_);
    }

    return value;
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

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
#include <srslib_framework/localization/map/logical/exception/UnexpectedGeometryException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedNumberOfPointsException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedValueException.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods
const string LogicalMapFactory::KEYWORD_BOUNDARY = "boundary";
const string LogicalMapFactory::KEYWORD_COORDINATES = "coordinates";
const string LogicalMapFactory::KEYWORD_COST_AREA = "cost_area";
const string LogicalMapFactory::KEYWORD_EDGE = "edge";
const string LogicalMapFactory::KEYWORD_FEATURE_COLLECTION = "FeatureCollection";
const string LogicalMapFactory::KEYWORD_FEATURES = "features";
const string LogicalMapFactory::KEYWORD_GEOMETRY = "geometry";
const string LogicalMapFactory::KEYWORD_GRAPH = "graph";
const string LogicalMapFactory::KEYWORD_MAP = "map";
const string LogicalMapFactory::KEYWORD_MAX = "max";
const string LogicalMapFactory::KEYWORD_NULL = "null";
const string LogicalMapFactory::KEYWORD_OBSTACLE = "obstacle";
const string LogicalMapFactory::KEYWORD_PROPERTIES = "properties";
const string LogicalMapFactory::KEYWORD_PROPERTY_COST_AREA_COST = "cost";
const string LogicalMapFactory::KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_COST = "envelope_cost";
const string LogicalMapFactory::KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_SIZE = "envelope_size";
const string LogicalMapFactory::KEYWORD_PROPERTY_FEATURE_TYPE = "type";
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
const string LogicalMapFactory::KEYWORD_VERTEX = "vertex";
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

    ntEntry(jsonDocument);

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

    ntEntry(jsonDocument);

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
bool LogicalMapFactory::findCollection(YAML::Node root, YAML::Node& result)
{
    if (root.IsSequence())
    {
        for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
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
        if (root[KEYWORD_TYPE])
        {
            string value = root[KEYWORD_TYPE].as<string>();
            if (value == KEYWORD_FEATURE_COLLECTION)
            {
                result = root[KEYWORD_FEATURES];
                return !result.IsNull();
            }
        }
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool LogicalMapFactory::findEntityInCollection(YAML::Node root, string entityType, YAML::Node& result)
{
    if (root.IsSequence())
    {
        for (YAML::const_iterator it = root.begin(); it != root.end(); ++it)
        {
            YAML::Node element = *it;

            string type = findEntityType(element);
            if (entityType == type)
            {
                result = element;
                return true;
            }
        }
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
string LogicalMapFactory::findEntityType(YAML::Node root)
{
    YAML::Node type = root[KEYWORD_PROPERTIES][KEYWORD_PROPERTY_FEATURE_TYPE];

    if (type)
    {
        return type.as<string>();
    }

    return "";
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntities(YAML::Node root)
{
    // Go through all the obstacles in the collection
    for (auto entity : root)
    {
        string entityType = findEntityType(entity);
        if (entityType == KEYWORD_OBSTACLE)
        {
            ntEntityObstacle(entity);
        }
        else if (entityType == KEYWORD_COST_AREA)
        {
            ntEntityCostArea(entity);
        }
        else if (entityType == KEYWORD_WEIGHT_AREA)
        {
            ntEntityWeightArea(entity);
        }
        else if (entityType == KEYWORD_EDGE)
        {
            ntEntityEdge(entity);
        }
        else if (entityType == KEYWORD_GRAPH)
        {
            ntEntityGraph(entity);
        }
        else if (entityType == KEYWORD_VERTEX)
        {
            ntEntityVertex(entity);
        }
        else
        {
            throw UnexpectedFeatureException(metadata_, entityType);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityBoundary(YAML::Node root)
{
    YAML::Node properties = root[KEYWORD_PROPERTIES];

    double envelopeSize = ntValueDouble(properties[KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_SIZE], false);
    Grid2d::BaseType envelopeCost = ntValueCost(properties[KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_COST], false);

    double widthM = map_->getWidthM();
    double heightM = map_->getHeightM();
    double resolution = map_->getResolution();

    vector<Pose<>> coordinates = ntGeometry(root[KEYWORD_GEOMETRY], 2, 2);

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
void LogicalMapFactory::ntEntityCostArea(YAML::Node root)
{
    YAML::Node properties = root[KEYWORD_PROPERTIES];

    Grid2d::BaseType cost = ntValueCost(properties[KEYWORD_PROPERTY_COST_AREA_COST], true);

    vector<Pose<>> coordinates = ntGeometry(root[KEYWORD_GEOMETRY], 2, 2);

    Pose<> bl = coordinates[0];
    Pose<> tr = coordinates[1];

    // Add the static obstacle
    addRectangleCost(bl, abs(tr.x - bl.x), abs(tr.y - bl.y), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityEdge(YAML::Node root)
{
    ntGeometryNull(root[KEYWORD_GEOMETRY]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityGraph(YAML::Node root)
{
    ntGeometryNull(root[KEYWORD_GEOMETRY]);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityMap(YAML::Node root)
{
    YAML::Node properties = root[KEYWORD_PROPERTIES];

    metadata_.origin = ntValuePoint(properties[KEYWORD_PROPERTY_MAP_ORIGIN], false);
    metadata_.resolution = ntValueDouble(properties[KEYWORD_PROPERTY_MAP_RESOLUTION], false);
    metadata_.widthM = ntValueDouble(properties[KEYWORD_PROPERTY_MAP_WIDTH], false);
    metadata_.heightM = ntValueDouble(properties[KEYWORD_PROPERTY_MAP_HEIGHT], false);

    map_ = new LogicalMap(metadata_);
    metadata_ = map_->getMetadata();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityObstacle(YAML::Node root)
{
    YAML::Node properties = root[KEYWORD_PROPERTIES];

    double envelopeSize = ntValueDouble(properties[KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_SIZE], false);
    Grid2d::BaseType envelopeCost = ntValueCost(properties[KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_COST], false);

    vector<Pose<>> coordinates = ntGeometry(root[KEYWORD_GEOMETRY], 2, 2);

    Pose<> bl = coordinates[0];
    Pose<> tr = coordinates[1];

    // Add the static obstacle
    addStaticObstacle(bl, abs(tr.x - bl.x), abs(tr.y - bl.y), envelopeSize, envelopeCost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityVertex(YAML::Node root)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityWeightArea(YAML::Node root)
{
    YAML::Node properties = root[KEYWORD_PROPERTIES];

    Grid2d::BaseType northCost = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHT_AREA_NORTH], false);
    Grid2d::BaseType eastCost = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHT_AREA_EAST], false);
    Grid2d::BaseType southCost = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHT_AREA_SOUTH], false);
    Grid2d::BaseType westCost = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHT_AREA_WEST], false);

    vector<Pose<>> coordinates = ntGeometry(root[KEYWORD_GEOMETRY], 1, 2);
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
void LogicalMapFactory::ntEntry(YAML::Node root)
{
    YAML::Node features;
    if (findCollection(root, features))
    {
        YAML::Node mapNode;
        if (findEntityInCollection(features, KEYWORD_MAP, mapNode))
        {
            ntEntityMap(mapNode);
        }
        else
        {
            throw FeatureNotFoundException(metadata_, KEYWORD_MAP);
        }

        YAML::Node boundaryNode;
        if (findEntityInCollection(features, KEYWORD_BOUNDARY, boundaryNode))
        {
            ntEntityBoundary(boundaryNode);
        }

        YAML::Node entitiesNode;
        if (findCollection(features, entitiesNode))
        {
            ntEntities(entitiesNode);
        }
    }
    else
    {
        throw FeaturesNotFoundException(metadata_);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
vector<Pose<>> LogicalMapFactory::ntGeometry(YAML::Node root, int minNumber, int maxNumber)
{
    vector<Pose<>> coordinates;

    if (root.IsScalar())
    {
        string value = root.as<string>();
        throw UnexpectedValueException(metadata_, value);
    }
    else
    {
        YAML::Node typeNode = root[KEYWORD_TYPE];
        if (typeNode)
        {
            YAML::Node coordinatesNode = root[KEYWORD_COORDINATES];

            string typeString = typeNode.as<string>();
            if (typeString == KEYWORD_TYPE_POINT)
            {
                coordinates.push_back(ntValuePoint(coordinatesNode, true));
            }
            else if (typeString == KEYWORD_TYPE_MULTIPOINT)
            {
                vector<Pose<>> points = ntValueMultiPoint(coordinatesNode, true);
                coordinates.insert(coordinates.end(), points.begin(), points.end());
            }
            else
            {
                throw GeoJsonTypeUnsupportedException(metadata_, typeString);
            }
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
void LogicalMapFactory::ntGeometryNull(YAML::Node root)
{
    if (root.IsScalar())
    {
        string value = root.as<string>();
        if (value != KEYWORD_NULL)
        {
            throw UnexpectedValueException(metadata_, value);
        }
    }
    else
    {
        throw UnexpectedGeometryException(metadata_);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
Grid2d::BaseType LogicalMapFactory::ntValueCost(YAML::Node root, bool required)
{
    int value = Grid2d::PAYLOAD_MIN;

    if (root)
    {
        string cost = root.as<string>();

        try
        {
            value = cost != KEYWORD_MAX ? root.as<int>() : Grid2d::PAYLOAD_MAX;
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
double LogicalMapFactory::ntValueDouble(YAML::Node root, bool required)
{
    double value = 0.0;

    if (root)
    {
        try
        {
            value = root.as<double>();
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
vector<Pose<>> LogicalMapFactory::ntValueMultiPoint(YAML::Node root, bool required)
{
    vector<Pose<>> value;

    if (root)
    {
        try
        {
            for (auto point : root)
            {
                value.push_back(ntValuePoint(point, true));
            }
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

////////////////////////////////////////////////////////////////////////////////////////////////////
Pose<> LogicalMapFactory::ntValuePoint(YAML::Node root, bool required)
{
    Pose<> value = Pose<>::ZERO;

    if (root)
    {
        try
        {
            value = root.as<Pose<>>();
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
  static bool decode(const Node& root, srs::Pose<double>& rhs)
  {
      // Check if it is a simple or full pose (without theta or with theta)
      if (root.size() == 2 || root.size() == 3)
      {
          rhs.x = root[0].as<double>();
          rhs.y = root[1].as<double>();

          // If theta is expected
          if (root.size() == 3)
          {
              rhs.theta = root[2].as<double>();
          }

          return true;
      }

      return false;
  }
};

} // namespace YAML

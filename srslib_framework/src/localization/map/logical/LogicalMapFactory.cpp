#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>

#include <algorithm>
using namespace std;

#include <srslib_framework/math/MeasurementMath.hpp>
#include <srslib_framework/math/PoseMath.hpp>
#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/exception/yaml/FailedToParseJsonException.hpp>
#include <srslib_framework/localization/map/logical/exception/CostExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/DoubleExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/FeatureExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/FeaturesExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/GeoJsonTypeUnsupportedException.hpp>
#include <srslib_framework/localization/map/logical/exception/IncompatibleLanguageVersionException.hpp>
#include <srslib_framework/localization/map/logical/exception/InvalidCostValueException.hpp>
#include <srslib_framework/localization/map/logical/exception/InvalidMapNoteFlagException.hpp>
#include <srslib_framework/localization/map/logical/exception/MapNoteExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/PoseExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/PropertiesExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/PropertyExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/StringExpectedException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedFeatureException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedGeometryException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedNumberOfPointsException.hpp>
#include <srslib_framework/localization/map/logical/exception/UnexpectedValueException.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public constants
const string LogicalMapFactory::LANGUAGE_VERSION = "1.2";

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private constants
const string LogicalMapFactory::KEYWORD_BOUNDARY = "boundary";
const string LogicalMapFactory::KEYWORD_COORDINATES = "coordinates";
const string LogicalMapFactory::KEYWORD_COST_AREA = "cost_area";
const string LogicalMapFactory::KEYWORD_EDGE = "edge";
const string LogicalMapFactory::KEYWORD_FEATURE_COLLECTION = "FeatureCollection";
const string LogicalMapFactory::KEYWORD_FEATURES = "features";
const string LogicalMapFactory::KEYWORD_GEOMETRY = "geometry";
const string LogicalMapFactory::KEYWORD_GRAPH = "graph";
const string LogicalMapFactory::KEYWORD_ID = "id";
const string LogicalMapFactory::KEYWORD_LABELED_AREA = "labeled_area";
const string LogicalMapFactory::KEYWORD_MAP = "map";
const string LogicalMapFactory::KEYWORD_MAX = "max";
const string LogicalMapFactory::KEYWORD_OBSTACLE = "obstacle";
const string LogicalMapFactory::KEYWORD_PROPERTIES = "properties";
const string LogicalMapFactory::KEYWORD_PROPERTY_COST_AREA_COST = "cost";
const string LogicalMapFactory::KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_COST = "envelope_cost";
const string LogicalMapFactory::KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_SIZE = "envelope_size";
const string LogicalMapFactory::KEYWORD_PROPERTY_FEATURE_OBJECT = "object";
const string LogicalMapFactory::KEYWORD_PROPERTY_LABEL_AREA_LABEL = "label";
const string LogicalMapFactory::KEYWORD_PROPERTY_LABEL_AREA_NOTES = "notes";
const string LogicalMapFactory::KEYWORD_PROPERTY_LANGUAGE_VERSION = "language_version";
const string LogicalMapFactory::KEYWORD_PROPERTY_MAP_HEIGHT = "height";
const string LogicalMapFactory::KEYWORD_PROPERTY_MAP_ORIGIN = "origin";
const string LogicalMapFactory::KEYWORD_PROPERTY_MAP_RESOLUTION = "resolution";
const string LogicalMapFactory::KEYWORD_PROPERTY_MAP_WIDTH = "width";
const string LogicalMapFactory::KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_COST = "envelope_cost";
const string LogicalMapFactory::KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_SIZE = "envelope_size";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHTED_AREA_EAST = "e";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHTED_AREA_NORTH = "n";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHTED_AREA_NORTH_EAST = "ne";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHTED_AREA_NORTH_WEST = "nw";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHTED_AREA_SOUTH = "s";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHTED_AREA_SOUTH_EAST = "se";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHTED_AREA_SOUTH_WEST = "sw";
const string LogicalMapFactory::KEYWORD_PROPERTY_WEIGHTED_AREA_WEST = "w";
const string LogicalMapFactory::KEYWORD_TYPE = "type";
const string LogicalMapFactory::KEYWORD_TYPE_POINT = "Point";
const string LogicalMapFactory::KEYWORD_TYPE_MULTIPOINT = "MultiPoint";
const string LogicalMapFactory::KEYWORD_VERTEX = "vertex";
const string LogicalMapFactory::KEYWORD_WEIGHTED_AREA = "weighted_area";

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromGrid2d(WeightedGrid2d* logical, Pose<> origin, double resolution)
{
    metadata_ = LogicalMetadata(logical->getWidth(), logical->getHeight(),
        origin, resolution, "");

    map_ = new LogicalMap(metadata_, logical);

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromJsonFile(string jsonFilename)
{
    YAML::Node jsonDocument;
    try
    {
        jsonDocument = YAML::LoadFile(jsonFilename);
    }
    catch (exception& e)
    {
        throw FailedToOpenFileException(jsonFilename);
    }

    map_ = nullptr;
    metadata_ = LogicalMetadata();
    metadata_.logicalFilename = jsonFilename;

    ntEntry(jsonDocument);

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromString(string geoJson)
{
    YAML::Node jsonDocument;
    try
    {
        jsonDocument = YAML::LoadFile(geoJson);
    }
    catch (exception& e)
    {
        throw FailedToParseJsonException(geoJson);
    }

    map_ = nullptr;
    metadata_ = LogicalMetadata();

    ntEntry(jsonDocument);

    return map_;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addCostArea(Pose<> origin, double widthM, double heightM,
    WeightedGrid2d::BaseType cost)
{
    unsigned int c0;
    unsigned int r0;

    unsigned int widthCells;
    unsigned int heightCells;

    calculateArea(origin, widthM, heightM, c0, r0, widthCells, heightCells);

    widthCells = max<unsigned int>(1, widthCells);
    heightCells = max<unsigned int>(1, heightCells);

    for (unsigned int r = r0; r < r0 + heightCells; ++r)
    {
        for (unsigned int c = c0; c < c0 + widthCells; ++c)
        {
            if (map_->isWithinBounds(c, r))
            {
                map_->costMax(c, r, cost);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addLabelArea(Pose<> origin, double widthM, double heightM,
    string label, shared_ptr<MapNotes> notes)
{
    unsigned int c0;
    unsigned int r0;

    unsigned int widthCells;
    unsigned int heightCells;

    calculateArea(origin, widthM, heightM, c0, r0, widthCells, heightCells);

    Rectangle surface(c0, r0, c0 + widthCells, r0 + heightCells);
    map_->addLabeledArea(surface, label, notes);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addObstacleArea(Pose<> origin, double widthM, double heightM,
    double sizeEnvelopeM, WeightedGrid2d::BaseType costEnvelope)
{
    // First add the envelope, if specified
    if (sizeEnvelopeM > 0.0 && costEnvelope > 0)
    {
        addCostArea(PoseMath::add(origin, Pose<>(-sizeEnvelopeM, -sizeEnvelopeM)),
            widthM + 2 * sizeEnvelopeM, heightM + 2 * sizeEnvelopeM,
            costEnvelope);
    }

    // Add the static obstacle
    addCostArea(origin, widthM, heightM, WeightedGrid2d::PAYLOAD_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::addWeightArea(Pose<> origin, double widthM, double heightM,
    WeightedGrid2d::BaseType north, WeightedGrid2d::BaseType northEast,
    WeightedGrid2d::BaseType east, WeightedGrid2d::BaseType southEast,
    WeightedGrid2d::BaseType south, WeightedGrid2d::BaseType southWest,
    WeightedGrid2d::BaseType west, WeightedGrid2d::BaseType northWest)
{
    unsigned int c0;
    unsigned int r0;

    unsigned int widthCells;
    unsigned int heightCells;

    calculateArea(origin, widthM, heightM, c0, r0, widthCells, heightCells);

    widthCells = max<unsigned int>(1, widthCells);
    heightCells = max<unsigned int>(1, heightCells);

    for (unsigned int r = r0; r < r0 + heightCells; ++r)
    {
        for (unsigned int c = c0; c < c0 + widthCells; ++c)
        {
            if (map_->isWithinBounds(c, r))
            {
                map_->setWeights(c, r, north, northEast,
                    east, southEast,
                    south, southWest,
                    west, northWest);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::calculateArea(Pose<> origin, double widthM, double heightM,
    unsigned int& c0, unsigned int& r0, unsigned int& widthCells, unsigned int& heightCells)
{
    Pose<> newOrigin = PoseMath::subtract(origin, metadata_.origin);

    double newWidthM = widthM;
    double x = newOrigin.x;
    if (x < 0)
    {
        newWidthM += x;
        newWidthM = newWidthM > 0 ? newWidthM : 0;
        x = 0;
    }

    double newHeightM = heightM;
    double y = newOrigin.y;
    if (y < 0)
    {
        newHeightM += y;
        newHeightM = newHeightM > 0 ? newHeightM : 0;
        y = 0;
    }

    c0 = MeasurementMath::m2Cells(x, metadata_.resolution);
    r0 = MeasurementMath::m2Cells(y, metadata_.resolution);

    widthCells = MeasurementMath::m2Cells(newWidthM, metadata_.resolution);
    widthCells = max<unsigned int>(0, widthCells);

    heightCells = MeasurementMath::m2Cells(newHeightM, metadata_.resolution);
    heightCells = max<unsigned int>(0, heightCells);
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

            string type = findEntityObject(element);
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
string LogicalMapFactory::findEntityObject(YAML::Node root)
{
    YAML::Node object = root[KEYWORD_PROPERTIES][KEYWORD_PROPERTY_FEATURE_OBJECT];

    if (object)
    {
        return object.as<string>();
    }

    return "";
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntities(YAML::Node root)
{
    // Go through all the obstacles in the collection
    for (auto entity : root)
    {
        string entityType = findEntityObject(entity);
        if (entityType == KEYWORD_COST_AREA)
        {
            ntEntityCostArea(entity);
        }
        else if (entityType == KEYWORD_EDGE)
        {
            ntEntityEdge(entity);
        }
        else if (entityType == KEYWORD_GRAPH)
        {
            ntEntityGraph(entity);
        }
        else if (entityType == KEYWORD_LABELED_AREA)
        {
            ntEntityLabelArea(entity);
        }
        else if (entityType == KEYWORD_OBSTACLE)
        {
            ntEntityObstacle(entity);
        }
        else if (entityType == KEYWORD_VERTEX)
        {
            ntEntityVertex(entity);
        }
        else if (entityType == KEYWORD_WEIGHTED_AREA)
        {
            ntEntityWeightArea(entity);
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
    WeightedGrid2d::BaseType envelopeCost = ntValueCost(properties[KEYWORD_PROPERTY_BOUNDARY_ENVELOPE_COST], false);

    double widthM = map_->getWidthM();
    double heightM = map_->getHeightM();

    vector<Pose<>> coordinates = ntGeometry(root[KEYWORD_GEOMETRY], 2, 2);

    Pose<> p1 = coordinates[0];
    Pose<> p2 = coordinates[1];

    // Add the bottom border
    addObstacleArea(Pose<>::ZERO, widthM, p1.y, envelopeSize, envelopeCost);

    // Add the left border
    addObstacleArea(Pose<>(0, p1.y), p1.x, heightM - p1.y, envelopeSize, envelopeCost);

    // Add the top border
    Pose<> t0 = Pose<>(p1.x, p2.y);
    addObstacleArea(Pose<>(p1.x, t0.y), widthM - p1.x, heightM - t0.y, envelopeSize, envelopeCost);

    // Add the right border
    t0 = Pose<>(p2.x, p1.y);
    addObstacleArea(t0, abs(widthM - t0.x), abs(t0.y - p2.y), envelopeSize, envelopeCost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityCostArea(YAML::Node root)
{
    YAML::Node properties = root[KEYWORD_PROPERTIES];

    WeightedGrid2d::BaseType cost = ntValueCost(properties[KEYWORD_PROPERTY_COST_AREA_COST], true);

    vector<Pose<>> coordinates = ntGeometry(root[KEYWORD_GEOMETRY], 2, 2);

    Pose<> p1 = coordinates[0];
    Pose<> p2 = coordinates[1];

    // Add the cost area
    addCostArea(p1, abs(p1.x - p2.x), abs(p1.y - p2.y), cost);
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
void LogicalMapFactory::ntEntityLabelArea(YAML::Node root)
{
    YAML::Node properties = ntProperties(root);

    string label = ntValueString(properties[KEYWORD_PROPERTY_LABEL_AREA_LABEL], true);

    shared_ptr<MapNotes> notes = ntValueMapNotes(properties[KEYWORD_PROPERTY_LABEL_AREA_NOTES], true);

    vector<Pose<>> coordinates = ntGeometry(root[KEYWORD_GEOMETRY], 2, 2);

    Pose<> p1 = coordinates[0];
    Pose<> p2 = coordinates[1];

    // Add the label area
    addLabelArea(p1, abs(p1.x - p2.x), abs(p1.y - p2.y), label, notes);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityMap(YAML::Node root)
{
    YAML::Node properties = root[KEYWORD_PROPERTIES];

    metadata_.origin = ntValuePoint(properties[KEYWORD_PROPERTY_MAP_ORIGIN], false);
    metadata_.resolution = ntValueDouble(properties[KEYWORD_PROPERTY_MAP_RESOLUTION], false);
    metadata_.widthM = ntValueDouble(properties[KEYWORD_PROPERTY_MAP_WIDTH], false);
    metadata_.heightM = ntValueDouble(properties[KEYWORD_PROPERTY_MAP_HEIGHT], false);

    metadata_.widthCells = MeasurementMath::m2Cells(metadata_.widthM, metadata_.resolution);
    metadata_.heightCells = MeasurementMath::m2Cells(metadata_.heightM, metadata_.resolution);

    WeightedGrid2d* grid = new WeightedGrid2d(metadata_.widthCells, metadata_.heightCells);

    map_ = new LogicalMap(metadata_, grid);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityObstacle(YAML::Node root)
{
    YAML::Node properties = root[KEYWORD_PROPERTIES];

    double envelopeSize = ntValueDouble(properties[KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_SIZE], false);
    WeightedGrid2d::BaseType envelopeCost = ntValueCost(properties[KEYWORD_PROPERTY_OBSTACLE_ENVELOPE_COST], false);

    vector<Pose<>> coordinates = ntGeometry(root[KEYWORD_GEOMETRY], 2, 2);

    Pose<> p1 = coordinates[0];
    if (coordinates.size() == 2)
    {
        Pose<> p2 = coordinates[1];
        addObstacleArea(p1, abs(p1.x - p2.x), abs(p1.y - p2.y), envelopeSize, envelopeCost);
    }
    else if (coordinates.size() == 1)
    {
        addObstacleArea(p1, metadata_.resolution, metadata_.resolution, envelopeSize, envelopeCost);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityVertex(YAML::Node root)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntityWeightArea(YAML::Node root)
{
    YAML::Node properties = ntProperties(root);

    WeightedGrid2d::BaseType north = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHTED_AREA_NORTH], false);
    WeightedGrid2d::BaseType northEast = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHTED_AREA_NORTH_EAST], false);
    WeightedGrid2d::BaseType east = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHTED_AREA_EAST], false);
    WeightedGrid2d::BaseType southEast = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHTED_AREA_SOUTH_EAST], false);
    WeightedGrid2d::BaseType south = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHTED_AREA_SOUTH], false);
    WeightedGrid2d::BaseType southWest = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHTED_AREA_SOUTH_WEST], false);
    WeightedGrid2d::BaseType west = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHTED_AREA_WEST], false);
    WeightedGrid2d::BaseType northWest = ntValueCost(properties[KEYWORD_PROPERTY_WEIGHTED_AREA_NORTH_WEST], false);

    vector<Pose<>> coordinates = ntGeometry(root[KEYWORD_GEOMETRY], 1, 2);
    Pose<> p1 = coordinates[0];
    if (coordinates.size() == 2)
    {
        Pose<> p2 = coordinates[1];
        addWeightArea(p1, abs(p1.x - p2.x), abs(p1.y - p2.y),
            north, northEast,
            east, southEast,
            south, southWest,
            west, northWest);
    }
    else if (coordinates.size() == 1)
    {
        addWeightArea(p1, metadata_.resolution, metadata_.resolution,
            north, northEast,
            east, southEast,
            south, southWest,
            west, northWest);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntEntry(YAML::Node root)
{
    // Read the global properties of the logical map file, and
    // check the language versions
    ntZeroLevelProperties(root);

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
            throw FeatureExpectedException(metadata_, KEYWORD_MAP);
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
        throw FeaturesExpectedException(metadata_);
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
    if (!root.IsNull()) {
        throw UnexpectedGeometryException(metadata_);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
YAML::Node LogicalMapFactory::ntProperties(YAML::Node root)
{
    YAML::Node properties = root[KEYWORD_PROPERTIES];

    if (!properties)
    {
        string id = root[KEYWORD_ID].as<string>();

        throw PropertiesExpectedException(metadata_, id);
    }

    return properties;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
WeightedGrid2d::BaseType LogicalMapFactory::ntValueCost(YAML::Node root, bool required)
{
    int value = WeightedGrid2d::PAYLOAD_MIN;

    if (root)
    {
        string cost = root.as<string>();

        try
        {
            value = cost != KEYWORD_MAX ? root.as<int>() : WeightedGrid2d::PAYLOAD_MAX;
        }
        catch (exception& e)
        {
            throw CostExpectedException(metadata_);
        }

        if (value < WeightedGrid2d::PAYLOAD_MIN || value > WeightedGrid2d::PAYLOAD_MAX)
        {
            throw InvalidCostValueException(metadata_, value,
                WeightedGrid2d::PAYLOAD_MIN, WeightedGrid2d::PAYLOAD_MAX);
        }
    }
    else if (required)
    {
        throw CostExpectedException(metadata_);
    }

    return static_cast<WeightedGrid2d::BaseType>(value);
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
shared_ptr<MapNotes> LogicalMapFactory::ntValueMapNotes(YAML::Node root, bool required)
{
    shared_ptr<MapNotes> value = shared_ptr<MapNotes>(new MapNotes());

    if (root)
    {
        for (YAML::const_iterator flag = root.begin(); flag != root.end(); ++flag)
        {
            string noteType = flag->first.as<string>();
            string noteValue = flag->second.as<string>();
            if (!value->add(noteType, noteValue))
            {
                throw InvalidMapNoteFlagException(metadata_, noteType, noteValue);
            }
        }
    }
    else if (required)
    {
        throw MapNoteExpectedException(metadata_);
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

////////////////////////////////////////////////////////////////////////////////////////////////////
string LogicalMapFactory::ntValueString(YAML::Node root, bool required)
{
    string value = "";

    if (root)
    {
        try
        {
            value = root.as<string>();
        }
        catch (exception& e)
        {
            throw StringExpectedException(metadata_);
        }
    }
    else if (required)
    {
        throw StringExpectedException(metadata_);
    }

    return value;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMapFactory::ntZeroLevelProperties(YAML::Node root)
{
    YAML::Node properties = root[KEYWORD_PROPERTIES];

    if (!properties)
    {
        throw PropertiesExpectedException(metadata_, "Level zero");
    }

    string languageVersion = ntValueString(properties[KEYWORD_PROPERTY_LANGUAGE_VERSION], true);
    if (LANGUAGE_VERSION != languageVersion) {
        throw IncompatibleLanguageVersionException(metadata_, LANGUAGE_VERSION, languageVersion);
    }
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

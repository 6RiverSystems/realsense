#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>

#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>

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

    LogicalMap* logicalMap = LogicalMapFactory::synthesizeMap(jsonDocument);
    logicalMap->setLogicalFilename(jsonFilename);

    return logicalMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromMetadata(LogicalMetadata metadata)
{
    return LogicalMapFactory::fromJsonFile(metadata.logicalFilename);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::synthesizeMap(YAML::Node documentNode)
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
    }

    return logicalMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::nonTerminalMap(YAML::Node mapNode)
{
    double resolution = mapNode["properties"]["resolution"].as<double>();
    Pose<> origin = mapNode["properties"]["origin"].as<Pose<>>();

    Pose<> c0 = mapNode["geometry"]["coordinates"][0].as<Pose<double>>();
    Pose<> c1 = mapNode["geometry"]["coordinates"][1].as<Pose<double>>();
    Pose<> c2 = mapNode["geometry"]["coordinates"][2].as<Pose<double>>();
    Pose<> c3 = mapNode["geometry"]["coordinates"][3].as<Pose<double>>();

    LogicalMap* logicalMap = new LogicalMap(abs(c1.x - c0.x), abs(c2.y - c1.y), resolution);
    logicalMap->setOrigin(origin);

    return logicalMap;
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

} // namespace srs

////////////////////////////////////////////////////////////////////////////////////////////////////
namespace YAML {

////////////////////////////////////////////////////////////////////////////////////////////////////
template<>
struct convert<srs::Pose<double>>
{
  static bool decode(const Node& node, srs::Pose<double>& rhs)
  {
      if (node.size() == 2 || node.size() == 3)
      {
          rhs.x = node[0].as<double>();
          rhs.y = node[1].as<double>();
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

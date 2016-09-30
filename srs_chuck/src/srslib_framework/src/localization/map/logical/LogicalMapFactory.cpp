#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>

//#include <yaml-cpp/yaml.h>
//
//#include <srslib_framework/graph/grid2d/Grid2dLocation.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromJsonFile(string jsonFilename)
{
    return nullptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap* LogicalMapFactory::fromMetadata(LogicalMetadata metadata)
{
    return LogicalMapFactory::fromJsonFile(metadata.logicalFilename);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods


//////////////////////////////////////////////////////////////////////////////////////////////////////
//void MapFactory::map2Notes(Map* map, vector<int8_t>& notes)
//{
//    notes.clear();
//
//    Grid2d* grid = map->getGrid();
//    if (grid)
//    {
//        for (int row = 0; row < grid->getHeight(); row++)
//        {
//            for (int col = 0; col < grid->getWidth(); col++)
//            {
//                Grid2dLocation location = Grid2dLocation(col, row);
//
//                MapNote* note = reinterpret_cast<MapNote*>(grid->getNote(location));
//                int8_t flags = note ? note->getFlags() : 0;
//
//                notes.push_back(flags);
//            }
//        }
//    }
//}

} // namespace srs

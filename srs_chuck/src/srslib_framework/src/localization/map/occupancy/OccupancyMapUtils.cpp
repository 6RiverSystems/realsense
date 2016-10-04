#include <srslib_framework/localization/map/occupancy/OccupancyMapUtils.hpp>

#include <limits>

#include <yaml-cpp/yaml.h>

#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/occupancy/InvalidChannelNumberException.hpp>
#include <srslib_framework/localization/map/MapNote.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMapUtils::map2Occupancy(const OccupancyMap* map, vector<int8_t>& occupancy)
{
    occupancy.clear();

    Grid2d* grid = map->getGrid();
    if (grid)
    {
        for (int row = 0; row < grid->getHeight(); row++)
        {
            for (int col = 0; col < grid->getWidth(); col++)
            {
                Grid2dLocation location = Grid2dLocation(col, row);
                occupancy.push_back(static_cast<int8_t>(grid->getCost(location)));
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapUtils::occupancy2Map(const OccupancyMetadata& metadata,
    const vector<int8_t>& occupancy)
{
    OccupancyMap* occupancyMap = new OccupancyMap(metadata.widthCells, metadata.heightCells,
        metadata.resolution);

    auto occupancyIterator = occupancy.begin();

    for (int row = 0; row < metadata.heightCells; row++)
    {
        for (int col = 0; col < metadata.widthCells; col++)
        {
            float cost = (static_cast<float>(*occupancyIterator) / 100) * 255;
            occupancyMap->setCost(col, row, static_cast<unsigned int>(cost));

            occupancyIterator++;
        }
    }

    return occupancyMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

#include <srslib_framework/localization/map/MapAdapter.hpp>

#include <limits>

#include <yaml-cpp/yaml.h>

#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/occupancy/exception/InvalidChannelNumberException.hpp>
#include <srslib_framework/localization/map/MapNote.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
costmap_2d::Costmap2D* MapAdapter::toCostMap2D(BaseMap* map)
{
    unsigned int rows = map->getHeightCells();
    unsigned int columns = map->getWidthCells();

    costmap_2d::Costmap2D* costMap = new costmap_2d::Costmap2D(columns, rows,
        map->getResolution(), 0, 0);

    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < columns; col++)
        {
            costMap->setCost(col, row, map->getCost(col, row));
        }
    }

    return costMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

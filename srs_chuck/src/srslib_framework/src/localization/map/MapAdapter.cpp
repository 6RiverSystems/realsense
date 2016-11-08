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
void MapAdapter::costMap2D2Vector(const costmap_2d::Costmap2D* map, vector<int8_t>& occupancy)
{
    occupancy.resize(map->getSizeInCellsX() * map->getSizeInCellsY(), 0);

    unsigned char* value = map->getCharMap();
    for (int row = 0; row < map->getSizeInCellsY(); ++row)
    {
        for (int col = 0; col < map->getSizeInCellsX(); ++col)
        {
            occupancy[map->getIndex(col, row)] = *(value++);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
costmap_2d::Costmap2D* MapAdapter::map2CostMap2D(BaseMap* map)
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
void MapAdapter::occupancyMap2Vector(const OccupancyMap* map, vector<int8_t>& occupancy)
{
    occupancy.clear();

    Grid2d* grid = map->getGrid();
    if (grid)
    {
        for (int row = 0; row < grid->getHeight(); row++)
        {
            for (int col = 0; col < grid->getWidth(); col++)
            {
                Grid2d::BaseType cost = grid->getPayload(Grid2d::Location(col, row));
                occupancy.push_back(map->cost2grayLevel(cost));
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
costmap_2d::Costmap2D* MapAdapter::weights2CostMap2D(BaseMap* map, int orientation)
{
    unsigned int rows = map->getHeightCells();
    unsigned int columns = map->getWidthCells();

    costmap_2d::Costmap2D* costMap = new costmap_2d::Costmap2D(columns, rows,
        map->getResolution(), 0, 0);

    Grid2d::BaseType north;
    Grid2d::BaseType east;
    Grid2d::BaseType south;
    Grid2d::BaseType west;
    Grid2d::BaseType cost;

    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < columns; col++)
        {
            map->getWeights(col, row, north, east, south, west);

            switch (orientation)
            {
                case Grid2d::ORIENTATION_NORTH:
                    cost = north;
                    break;

                case Grid2d::ORIENTATION_EAST:
                    cost = east;
                    break;

                case Grid2d::ORIENTATION_SOUTH:
                    cost = south;
                    break;

                case Grid2d::ORIENTATION_WEST:
                    cost = west;
                    break;
            }

            costMap->setCost(col, row, cost);
        }
    }

    return costMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
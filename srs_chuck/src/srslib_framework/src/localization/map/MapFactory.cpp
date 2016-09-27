#include <srslib_framework/localization/map/MapFactory.hpp>

#include <costmap_2d/costmap_2d.h>

#include <srslib_framework/graph/grid2d/Grid2dLocation.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
Map* MapFactory::fromRosCostMap2D(costmap_2d::Costmap2DROS* rosCostMap,
    unsigned int obstacleThreshold)
{
    if (!rosCostMap)
    {
        return nullptr;
    }

    costmap_2d::Costmap2D* costMap = rosCostMap->getCostmap();

    unsigned int rows = costMap->getSizeInCellsY();
    unsigned int columns = costMap->getSizeInCellsX();

    Map* map = new Map(columns, rows, costMap->getResolution());

    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < columns; col++)
        {
            unsigned int cost = static_cast<unsigned int>(costMap->getCost(col, row));

            if (cost >= obstacleThreshold)
            {
                map->setObstruction(col, row);
            }
            else
            {
                map->setCost(col, row, static_cast<unsigned int>(cost));
            }
        }
    }

    return map;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapFactory::map2Occupancy(Map* map, vector<int8_t>& occupancy)
{
    occupancy.clear();

    Grid2d* grid = map->getGrid();
    if (grid)
    {
        int8_t maxValue = numeric_limits<int8_t>::max();

        for (int row = 0; row < grid->getHeight(); row++)
        {
            for (int col = 0; col < grid->getWidth(); col++)
            {
                Grid2dLocation location = Grid2dLocation(col, row);

                float cost = (static_cast<float>(grid->getCost(location)) / (2.0 * maxValue)) * 100;

                // This is only for visualization purposes as
                // a static obstacle information is carried by the map note
                MapNote* note = reinterpret_cast<MapNote*>(grid->getNote(location));
                if (note && note->staticObstacle())
                {
                    cost = 100.0;
                }

                occupancy.push_back(static_cast<int8_t>(ceil(cost)));
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapFactory::map2Notes(Map* map, vector<int8_t>& notes)
{
    notes.clear();

    Grid2d* grid = map->getGrid();
    if (grid)
    {
        for (int row = 0; row < grid->getHeight(); row++)
        {
            for (int col = 0; col < grid->getWidth(); col++)
            {
                Grid2dLocation location = Grid2dLocation(col, row);

                MapNote* note = reinterpret_cast<MapNote*>(grid->getNote(location));
                int8_t flags = note ? note->getFlags() : 0;

                notes.push_back(flags);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

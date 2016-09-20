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
// Private methods

} // namespace srs

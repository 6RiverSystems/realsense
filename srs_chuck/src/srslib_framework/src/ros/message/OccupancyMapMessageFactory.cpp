#include <srslib_framework/ros/message/OccupancyMapMessageFactory.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMapMessageFactory::map2Vector(const OccupancyMap* map, vector<int8_t>& occupancy)
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
// Private methods

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap* OccupancyMapMessageFactory::vector2Map(const OccupancyMetadata& metadata,
    const vector<int8_t>& occupancy)
{
    OccupancyMap* map = new OccupancyMap(metadata);

    auto occupancyIterator = occupancy.begin();

    for (int row = 0; row < metadata.heightCells; row++)
    {
        for (int col = 0; col < metadata.widthCells; col++)
        {
            // Convert the 8 bit cost into an integer cost and store it
            int8_t grayLevel = *occupancyIterator;
            Grid2d::BaseType cost = map->grayLevel2Cost(static_cast<unsigned char>(grayLevel));
            map->setCost(col, row, cost);

            occupancyIterator++;
        }
    }

    return map;
}

} // namespace srs

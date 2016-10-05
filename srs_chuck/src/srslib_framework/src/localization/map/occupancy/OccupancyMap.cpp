#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(unsigned int widthCells, unsigned int heightCells, double resolution) :
        BaseMap(widthCells, heightCells, resolution)
{
    occupancyMetadata_.heightCells = heightCells;
    occupancyMetadata_.heightMm = getHeightMm();
    occupancyMetadata_.resolution = resolution;
    occupancyMetadata_.widthCells = widthCells;
    occupancyMetadata_.widthMm = getWidthMm();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::addCost(unsigned int c, unsigned int r, unsigned int cost)
{
    getGrid()->addCost(c, r, cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::setCost(unsigned int c, unsigned int r, unsigned int cost)
{
    Grid2dLocation location = Grid2dLocation(c, r);
    getGrid()->addValue(location, cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::setObstruction(unsigned int c, unsigned int r)
{
    Grid2dLocation location = Grid2dLocation(c, r);
    getGrid()->addValue(location, numeric_limits<unsigned int>::max(), nullptr);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

////////////////////////////////////////////////////////////////////////////////////////////////////
// Global operators

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const OccupancyMap& map)
{
    unsigned int rows = map.getHeightCells();
    unsigned int columns = map.getWidthCells();
    stream << "OccupancyMap (" << rows << "x" << columns << ")" << endl;

    for (int row = rows - 1; row >= 0; row--)
    {
        for (unsigned int col = 0; col < columns; col++)
        {
            unsigned int rawCost = map.getCost(col, static_cast<unsigned int>(row));

            char cost = rawCost > 0 ? '+' : '.';
            cost = rawCost == numeric_limits<unsigned int>::max() ? '#' : cost;

            stream << cost;
        }

        stream << endl;
    }

    return stream;
}

} // namespace srs

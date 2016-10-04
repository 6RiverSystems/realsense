#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(unsigned int widthCells, unsigned int heightCells, double resolution) :
        BaseMap(widthCells, heightCells, resolution)
{
    occupancyMetadata_.heightCells = getHeightCells();
    occupancyMetadata_.heightM = getHeightMeters();
    occupancyMetadata_.resolution = resolution;
    occupancyMetadata_.widthCells = getWidthCells();
    occupancyMetadata_.widthM = getWidthMeters();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::~OccupancyMap()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::setCost(int c, int r, unsigned int cost)
{
    Grid2dLocation location = Grid2dLocation(c, r);
    getGrid()->addValue(location, cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::setObstruction(int c, int r)
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
    int8_t maxValue = numeric_limits<unsigned int>::max();

    unsigned int rows = map.getHeightCells();
    unsigned int columns = map.getWidthCells();
    stream << "OccupancyMap (" << rows << "x" << columns << ")" << endl;

    for (unsigned int row = rows - 1; row >= 0; row--)
    {
        for (unsigned int col = 0; col < columns; col++)
        {
            unsigned int rawCost = map.getCost(col, row);

            char cost = rawCost > 0.0 ? '+' : '.';
            if (rawCost == maxValue)
            {
                cost = '#';
            }

            stream << cost;
        }

        stream << endl;
    }

    return stream;
}

} // namespace srs

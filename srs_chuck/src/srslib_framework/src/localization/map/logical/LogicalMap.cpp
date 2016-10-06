#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

#include <limits>

#include <srslib_framework/utils/Filesystem.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap::LogicalMap(double widthM, double heightM, double resolution) :
        BaseMap(widthM, heightM, resolution)
{
    logicalMetadata_.heightCells = getHeightCells();
    logicalMetadata_.heightMm = heightM;
    logicalMetadata_.resolution = resolution;
    logicalMetadata_.widthCells = getWidthCells();
    logicalMetadata_.widthMm = widthM;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::addCost(unsigned int c, unsigned int r, unsigned int cost)
{
    getGrid()->addCost(Grid2d::Location(c, r), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setCost(unsigned int c, unsigned int r, unsigned int cost)
{
    getGrid()->setCost(Grid2d::Location(c, r), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setObstruction(unsigned int c, unsigned int r)
{
    getGrid()->setCost(Grid2d::Location(c, r), numeric_limits<unsigned int>::max());
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

////////////////////////////////////////////////////////////////////////////////////////////////////
// Global operators

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const LogicalMap& map)
{
    unsigned int rows = map.getHeightCells();
    unsigned int columns = map.getWidthCells();
    stream << "LogicalMap (" << rows << "x" << columns << ")" << endl;

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

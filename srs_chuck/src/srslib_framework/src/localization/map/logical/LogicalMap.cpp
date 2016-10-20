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
    logicalMetadata_.heightM = getHeightM();
    logicalMetadata_.resolution = getResolution();
    logicalMetadata_.widthCells = getWidthCells();
    logicalMetadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap::LogicalMap(Grid2d* grid, double resolution) :
    BaseMap(grid, resolution)
{
    logicalMetadata_.heightCells = getHeightCells();
    logicalMetadata_.heightM = getHeightM();
    logicalMetadata_.resolution = getResolution();
    logicalMetadata_.widthCells = getWidthCells();
    logicalMetadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::maxCost(unsigned int c, unsigned int r, int cost)
{
    getGrid()->maxOnPayload(Grid2d::Location(c, r), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setCost(unsigned int c, unsigned int r, int cost)
{
    getGrid()->setPayload(Grid2d::Location(c, r), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setObstruction(unsigned int c, unsigned int r)
{
    getGrid()->setPayload(Grid2d::Location(c, r), Grid2d::PAYLOAD_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setWeights(unsigned int c, unsigned int r,
    int north, int east, int south, int west)
{
    getGrid()->setWeights(Grid2d::Location(c, r), north, east, south, west);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

////////////////////////////////////////////////////////////////////////////////////////////////////
// Global operators

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const LogicalMap& map)
{
    stream << "LogicalMap" << endl;
    stream << *map.getGrid() << endl;

    return stream;
}

} // namespace srs

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
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap::LogicalMap(Grid2d* grid, double resolution) :
    BaseMap(grid, resolution)
{
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap::LogicalMap(LogicalMetadata metadata) :
    BaseMap(metadata.widthM, metadata.heightM, metadata.resolution)
{
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();

    metadata_.loadTime = metadata.loadTime;
    metadata_.logicalFilename = metadata.logicalFilename;
    metadata_.origin = metadata.origin;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::maxCost(unsigned int c, unsigned int r, Grid2d::BaseType cost)
{
    getGrid()->maxOnPayload(Grid2d::Location(c, r), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const LogicalMap& map)
{
    stream << "LogicalMap" << endl;
    stream << *map.getGrid() << endl;

    return stream;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool operator==(const LogicalMap& lhs, const LogicalMap& rhs)
{
    return lhs.metadata_ == rhs.metadata_ &&
        operator==(static_cast<const BaseMap&>(lhs), static_cast<const BaseMap&>(rhs));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setCost(unsigned int c, unsigned int r, Grid2d::BaseType cost)
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
    Grid2d::BaseType north,
    Grid2d::BaseType east,
    Grid2d::BaseType south,
    Grid2d::BaseType west)
{
    getGrid()->setWeights(Grid2d::Location(c, r), north, east, south, west);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

} // namespace srs

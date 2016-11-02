#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

const int8_t OccupancyMap::COST_INT8_MAX = numeric_limits<int8_t>::max();
const unsigned char OccupancyMap::COST_UCHAR_MAX = numeric_limits<unsigned char>::max();

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(unsigned int widthCells, unsigned int heightCells, double resolution) :
        BaseMap(widthCells, heightCells, resolution)
{
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(Grid2d* grid, double resolution) :
        BaseMap(grid, resolution)
{
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(OccupancyMetadata metadata) :
        BaseMap(metadata.widthCells, metadata.heightCells, metadata.resolution)
{
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();

    metadata_.loadTime = metadata.loadTime;
    metadata_.negate = metadata.negate;
    metadata_.occupancyFilename = metadata.occupancyFilename;
    metadata_.origin = metadata.origin;
    metadata_.thresholdFree = metadata.thresholdFree;
    metadata_.thresholdOccupied = metadata.thresholdOccupied;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const OccupancyMap& map)
{
    stream << "OccupancyMap" << endl;
    stream << *map.getGrid() << endl;

    return stream;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
bool operator==(const OccupancyMap& lhs, const OccupancyMap& rhs)
{
    return lhs.metadata_ == rhs.metadata_ &&
        operator==(static_cast<const BaseMap&>(lhs), static_cast<const BaseMap&>(rhs));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::maxCost(unsigned int c, unsigned int r, Grid2d::BaseType cost)
{
    getGrid()->maxOnPayload(Grid2d::Location(c, r), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::setCost(unsigned int c, unsigned int r, Grid2d::BaseType cost)
{
    getGrid()->setPayload(Grid2d::Location(c, r), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::setObstruction(unsigned int c, unsigned int r)
{
    getGrid()->setPayload(Grid2d::Location(c, r), Grid2d::PAYLOAD_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

} // namespace srs

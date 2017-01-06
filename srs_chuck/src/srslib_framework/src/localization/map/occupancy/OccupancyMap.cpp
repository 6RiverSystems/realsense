#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

#include <srslib_framework/datastructure/Location.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

const int8_t OccupancyMap::COST_INT8_MAX = numeric_limits<int8_t>::max();
const unsigned char OccupancyMap::COST_UCHAR_MAX = numeric_limits<unsigned char>::max();

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(unsigned int widthCells, unsigned int heightCells,
    double resolution, Pose<> origin) :
        BaseMap(widthCells, heightCells, resolution, origin)
{
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(Grid2d* grid, double resolution, Pose<> origin) :
        BaseMap(grid, resolution, origin)
{
    metadata_.heightCells = getHeightCells();
    metadata_.heightM = getHeightM();
    metadata_.resolution = getResolution();
    metadata_.widthCells = getWidthCells();
    metadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(OccupancyMetadata metadata) :
        BaseMap(metadata.widthCells, metadata.heightCells, metadata.resolution, metadata.origin)
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
void OccupancyMap::maxCost(unsigned int cCells, unsigned int rCells, Grid2d::BaseType cost)
{
    getGrid()->maxOnPayload(Location(cCells, rCells), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::setCost(unsigned int cCells, unsigned int rCells, Grid2d::BaseType cost)
{
    getGrid()->setPayload(Location(cCells, rCells), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::setObstacle(unsigned int cCells, unsigned int rCells)
{
    getGrid()->setPayload(Location(cCells, rCells), Grid2d::PAYLOAD_MAX);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

} // namespace srs

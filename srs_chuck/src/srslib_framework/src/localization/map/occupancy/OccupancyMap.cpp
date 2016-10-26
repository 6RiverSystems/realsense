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
    occupancyMetadata_.heightCells = getHeightCells();
    occupancyMetadata_.heightM = getHeightM();
    occupancyMetadata_.resolution = getResolution();
    occupancyMetadata_.widthCells = getWidthCells();
    occupancyMetadata_.widthM = getWidthM();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(OccupancyMetadata metadata) :
            BaseMap(metadata.widthCells, metadata.heightCells, metadata.resolution)
{
    occupancyMetadata_.heightCells = getHeightCells();
    occupancyMetadata_.heightM = getHeightM();
    occupancyMetadata_.resolution = getResolution();
    occupancyMetadata_.widthCells = getWidthCells();
    occupancyMetadata_.widthM = getWidthM();

    occupancyMetadata_.loadTime = metadata.loadTime;
    occupancyMetadata_.negate = metadata.negate;
    occupancyMetadata_.occupancyFilename = metadata.occupancyFilename;
    occupancyMetadata_.origin = metadata.origin;
    occupancyMetadata_.thresholdFree = metadata.thresholdFree;
    occupancyMetadata_.thresholdOccupied = metadata.thresholdOccupied;
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
    return lhs.occupancyMetadata_ == rhs.occupancyMetadata_ &&
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

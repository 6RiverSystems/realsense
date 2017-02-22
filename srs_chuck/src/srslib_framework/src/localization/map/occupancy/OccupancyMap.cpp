#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

#include <srslib_framework/datastructure/Location.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(OccupancyMetadata metadata, SimpleGrid2d* grid) :
    BaseMap(grid, metadata.widthM, metadata.heightM, metadata.resolution, metadata.origin),
    metadata_(metadata)
{}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::costMax(unsigned int cCells, unsigned int rCells, SimpleGrid2d::BaseType cost)
{
    getGrid()->payloadMax(Location(cCells, rCells), cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::costSet(unsigned int cCells, unsigned int rCells, SimpleGrid2d::BaseType cost)
{
    getGrid()->payloadSet(Location(cCells, rCells), cost);
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
    if (&lhs == &rhs)
    {
        return true;
    }

    return lhs.metadata_ == rhs.metadata_ && *lhs.getGrid() == *rhs.getGrid();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

} // namespace srs

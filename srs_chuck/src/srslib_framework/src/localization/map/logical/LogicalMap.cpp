#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

#include <limits>

#include <srslib_framework/utils/Filesystem.hpp>
#include <srslib_framework/graph/grid2d/Grid2dLocation.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
LogicalMap::LogicalMap(double widthM, double heightM, double resolution) :
        BaseMap(widthM, heightM, resolution)
{
    logicalMetadata_.heightCells = getHeightCells();
    logicalMetadata_.heightM = heightM;
    logicalMetadata_.resolution = resolution;
    logicalMetadata_.widthCells = getWidthCells();
    logicalMetadata_.widthM = widthM;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setCost(unsigned int c, unsigned int r, unsigned int cost)
{
//    Grid2dLocation location = Grid2dLocation(c, r);
//    grid_->addValue(location, cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void LogicalMap::setObstruction(unsigned int c, unsigned int r)
{
//    Grid2dLocation location = Grid2dLocation(c, r);
//    void* note = reinterpret_cast<void*>(MapNote::instanceOf(MapNote::STATIC_OBSTACLE));
//
//    grid_->addValue(location, numeric_limits<unsigned int>::max(), note);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods
//

////////////////////////////////////////////////////////////////////////////////////////////////////
// Global operators

////////////////////////////////////////////////////////////////////////////////////////////////////
ostream& operator<<(ostream& stream, const LogicalMap& map)
{
//    int8_t maxValue = numeric_limits<int8_t>::max();
//
//    stream << "Map (" << map.grid_->getHeight() << "x" << map.grid_->getWidth() << ")" << endl;
//
//    for (int row = map.grid_->getHeight() - 1; row >= 0; row--)
//    {
//        for (int col = 0; col < map.grid_->getWidth(); col++)
//        {
//            Grid2dLocation location = Grid2dLocation(col, row);
//
//            MapNote* note = reinterpret_cast<MapNote*>(map.grid_->getNote(location));
//            float floatCost = (static_cast<float>(map.grid_->getCost(location)) / maxValue) * 100.0;
//
//            char cost = floatCost > 0.0 ? '+' : '.';
//
//            if (note && note->staticObstacle())
//            {
//                cost = '#';
//            }
//
//            stream << cost;
//        }
//
//        stream << endl;
//    }
//
    return stream;
}

} // namespace srs

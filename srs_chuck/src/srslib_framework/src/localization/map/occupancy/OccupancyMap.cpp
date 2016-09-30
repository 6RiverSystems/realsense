#include <srslib_framework/localization/map/occupancy/OccupancyMap.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap() :
        BaseMap()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(double widthC, double heightC, double resolution) :
        BaseMap(widthC, heightC, resolution)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::OccupancyMap(OccupancyMetadata metadata) :
        BaseMap(metadata)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
OccupancyMap::~OccupancyMap()
{
//    if (grid_)
//    {
//        // Find the map notes and remove them first, because Grid will not
//        // do it for us
//        for (int row = 0; row < grid_->getHeight(); row++)
//        {
//            for (int col = 0; col < grid_->getWidth(); col++)
//            {
//                Grid2dLocation location = Grid2dLocation(col, row);
//                MapNote* note = reinterpret_cast<MapNote*>(grid_->getNote(location));
//
//                if (note)
//                {
//                    delete note;
//                }
//            }
//        }
//
//        // Now deallocate the grid
//        delete grid_;
//    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::setCost(int c, int r, unsigned int cost)
{
//    Grid2dLocation location = Grid2dLocation(c, r);
//    grid_->addValue(location, cost);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void OccupancyMap::setObstruction(int c, int r)
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
ostream& operator<<(ostream& stream, const OccupancyMap& map)
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
//    return stream;
}

} // namespace srs

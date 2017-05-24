#include <srslib_framework/localization/map/MapAdapter.hpp>

#include <limits>

#include <yaml-cpp/yaml.h>
#include <costmap_2d/cost_values.h>

#include <srslib_framework/datastructure/Location.hpp>
#include <srslib_framework/exception/io/FailedToOpenFileException.hpp>
#include <srslib_framework/localization/map/occupancy/exception/InvalidChannelNumberException.hpp>

namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapAdapter::costMap2D2Vector(const costmap_2d::Costmap2D* costmap, vector<int8_t>& int8Vector)
{
    int8Vector.resize(costmap->getSizeInCellsX() * costmap->getSizeInCellsY(), 0);

    unsigned char* value = costmap->getCharMap();
    for (size_t row = 0; row < costmap->getSizeInCellsY(); ++row)
    {
        for (size_t col = 0; col < costmap->getSizeInCellsX(); ++col)
        {
            int8Vector[costmap->getIndex(col, row)] = *(value++);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
costmap_2d::Costmap2D* MapAdapter::map2CostMap2D(OccupancyMap* occupancy)
{
    size_t rows = occupancy->getHeightCells();
    size_t columns = occupancy->getWidthCells();

    costmap_2d::Costmap2D* costMap = new costmap_2d::Costmap2D(columns, rows,
        occupancy->getResolution(), occupancy->getOrigin().x, occupancy->getOrigin().y);

    for (size_t row = 0; row < rows; row++)
    {
        for (size_t col = 0; col < columns; col++)
        {
            SimpleGrid2d::BaseType cost = occupancy->getCost(col, row);
            cost = cost == SimpleGrid2d::PAYLOAD_NO_INFORMATION ? costmap_2d::NO_INFORMATION : cost;

            costMap->setCost(col, row, cost);
        }
    }

    return costMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
costmap_2d::Costmap2D* MapAdapter::map2CostMap2D(const LogicalMap* logical)
{
    unsigned int rows = logical->getHeightCells();
    unsigned int columns = logical->getWidthCells();

    costmap_2d::Costmap2D* costMap = new costmap_2d::Costmap2D(columns, rows,
        logical->getResolution(), logical->getOrigin().x, logical->getOrigin().y);

    for (size_t row = 0; row < rows; row++)
    {
        for (size_t col = 0; col < columns; col++)
        {
            costMap->setCost(col, row, logical->getCost(col, row));
        }
    }

    return costMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapAdapter::occupancyMap2AmclVector(const OccupancyMap* occupancy, vector<int8_t>& int8Vector)
{
    int8Vector.clear();

    for (int row = 0; row < occupancy->getHeightCells(); row++)
    {
        for (int col = 0; col < occupancy->getWidthCells(); col++)
        {
            SimpleGrid2d::BaseType cost = occupancy->getCost(col, row);

            int8_t adaptedCost = static_cast<int8_t>(cost);
            if (cost == SimpleGrid2d::PAYLOAD_NO_INFORMATION)
            {
                adaptedCost = -1;
            }
            else if (cost == SimpleGrid2d::PAYLOAD_MIN)
            {
                adaptedCost = 0;
            }
            else if (cost == SimpleGrid2d::PAYLOAD_MAX)
            {
                adaptedCost = 100;
            }

            int8Vector.push_back(adaptedCost);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
void MapAdapter::occupancyMap2Vector(const OccupancyMap* occupancy, vector<int8_t>& int8Vector)
{
    int8Vector.clear();

    for (int row = 0; row < occupancy->getHeightCells(); row++)
    {
        for (int col = 0; col < occupancy->getWidthCells(); col++)
        {
            int8_t cost = static_cast<int8_t>(occupancy->getCost(col, row));
            int8Vector.push_back(cost);
        }
    }
}

void MapAdapter::logicalMap2OccupancyVector(const LogicalMap* logical, vector<int8_t>& int8Vector)
{
    int8Vector.clear();

    for (int row = 0; row < logical->getHeightCells(); row++)
    {
        for (int col = 0; col < logical->getWidthCells(); col++)
        {
            uint32_t rawCost = static_cast<uint32_t>(logical->getCost(col, row));

            int8_t cost = static_cast<int8_t>(100 * (rawCost - WeightedGrid2d::PAYLOAD_MIN)
                / (WeightedGrid2d::PAYLOAD_MAX - WeightedGrid2d::PAYLOAD_MIN));

            int8Vector.push_back(cost);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
costmap_2d::Costmap2D* MapAdapter::weights2CostMap2D(LogicalMap* occupancy, int orientation)
{
    int rows = occupancy->getHeightCells();
    int columns = occupancy->getWidthCells();

    costmap_2d::Costmap2D* costMap = new costmap_2d::Costmap2D(columns, rows,
        occupancy->getResolution(), occupancy->getOrigin().x, occupancy->getOrigin().y);

    WeightedGrid2d::BaseType north;
    WeightedGrid2d::BaseType northEast;
    WeightedGrid2d::BaseType east;
    WeightedGrid2d::BaseType southEast;
    WeightedGrid2d::BaseType south;
    WeightedGrid2d::BaseType southWest;
    WeightedGrid2d::BaseType west;
    WeightedGrid2d::BaseType northWest;
    WeightedGrid2d::BaseType cost = 0;

    for (int row = 0; row < rows; row++)
    {
        for (int col = 0; col < columns; col++)
        {
            occupancy->getWeights(col, row,
                north, northEast,
                east, southEast,
                south, southWest,
                west, northWest);

            switch (orientation)
            {
                case WeightedGrid2d::ORIENTATION_NORTH:
                    cost = north;
                    break;

                case WeightedGrid2d::ORIENTATION_NORTH_EAST:
                    cost = northEast;
                    break;

                case WeightedGrid2d::ORIENTATION_EAST:
                    cost = east;
                    break;

                case WeightedGrid2d::ORIENTATION_SOUTH_EAST:
                    cost = southEast;
                    break;

                case WeightedGrid2d::ORIENTATION_SOUTH:
                    cost = south;
                    break;

                case WeightedGrid2d::ORIENTATION_SOUTH_WEST:
                    cost = southWest;
                    break;

                case WeightedGrid2d::ORIENTATION_WEST:
                    cost = west;
                    break;

                case WeightedGrid2d::ORIENTATION_NORTH_WEST:
                    cost = northWest;
                    break;
            }

            costMap->setCost(col, row, cost);
        }
    }

    return costMap;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs

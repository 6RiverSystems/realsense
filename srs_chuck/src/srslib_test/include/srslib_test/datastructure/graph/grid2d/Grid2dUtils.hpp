/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <iostream>
using namespace std;

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/map/MapStack.hpp>
#include <srslib_framework/localization/map/MapAdapter.hpp>
#include <srslib_framework/localization/map/logical/LogicalMapFactory.hpp>
#include <srslib_framework/localization/map/occupancy/OccupancyMapFactory.hpp>

namespace srs {
namespace test {

struct Grid2dUtils
{
    static void addRectanglePayload(Grid2d& grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        Grid2d::BaseType cost)
    {
        int deltaX = BasicMath::sgn<int>(xf - xi);
        int deltaY = BasicMath::sgn<int>(yf - yi);

        unsigned int c;
        unsigned int r = yi;
        do
        {
            c = xi;

            do
            {
                grid.maxOnPayload(Grid2d::Location(c, r), cost);
                c += deltaX;
            } while (c != (xf + deltaX));

            r += deltaY;
        } while (r != (yf + deltaY));
    }

    static void addObstacle(Grid2d& grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        int sizeEnvelopeCells = 0, Grid2d::BaseType costEnvelope = 0)
    {
        // First add the envelope, if specified
        if (sizeEnvelopeCells > 0 && costEnvelope > 0)
        {
            addRectanglePayload(grid,
                xi - sizeEnvelopeCells, yi - sizeEnvelopeCells,
                xf + sizeEnvelopeCells, yf + sizeEnvelopeCells,
                costEnvelope);
        }

        // Add the static obstacle
        addRectanglePayload(grid, xi, yi, xf, yf, Grid2d::PAYLOAD_MAX);
    }

    static void addWeights(Grid2d& grid,
        unsigned int xi, unsigned int yi, unsigned int xf, unsigned int yf,
        Grid2d::BaseType north,
        Grid2d::BaseType east,
        Grid2d::BaseType south,
        Grid2d::BaseType west)
    {
        int deltaX = BasicMath::sgn<int>(xf - xi);
        int deltaY = BasicMath::sgn<int>(yf - yi);

        unsigned int c;
        unsigned int r = yi;
        do
        {
            c = xi;

            do
            {
                grid.setWeights(Grid2d::Location(c, r), north, east, south, west);
                c += deltaX;
            } while (c != (xf + deltaX));

            r += deltaY;
        } while (r != (yf + deltaY));
    }

    static MapStack* grid2d2MapStack(Grid2d* grid, const Pose<> origin)
    {
        LogicalMapFactory logicalMapFactory;
        LogicalMap* logical = logicalMapFactory.fromGrid2d(grid, 1, origin);

        OccupancyMapFactory occupancyMapFactory;
        OccupancyMap* occupancy = occupancyMapFactory.fromGrid2d(grid, 1, origin);

        costmap_2d::Costmap2D* costMap2d = MapAdapter::map2CostMap2D(occupancy);

        return new MapStack(logical, occupancy, costMap2d);
    }

};

} // namespace test
} // namespace srs

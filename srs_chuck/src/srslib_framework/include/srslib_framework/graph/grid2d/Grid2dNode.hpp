/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef GRID2DNODE_HPP_
#define GRID2DNODE_HPP_

#include <srslib_framework/graph/grid2d/Grid2dLocation.hpp>

namespace srs {

struct Grid2dNode
{
    Grid2dNode(Grid2dLocation location, int cost) :
        location(location),
        cost(cost)
    {}

    friend ostream& operator<<(ostream& stream, const Grid2dNode* node)
    {
        stream << "Grid2dNode "<< hex << reinterpret_cast<long>(node) << dec << " {" << '\n';
        stream << "  l: " << node->location << " c: " << node->cost << '\n';
        stream << "}";
        return stream;
    }

    Grid2dLocation location;
    int cost;
};

} // namespace srs

#endif // GRID2DNODE_HPP_

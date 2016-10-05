/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef GRID2DNODE_HPP_
#define GRID2DNODE_HPP_

#include <srslib_framework/graph/grid2d/Grid2dLocation.hpp>

namespace srs {

class Grid2dNode
{
public:
    Grid2dNode(Grid2dLocation location, unsigned int cost, void* notes) :
        location(location),
        cost(cost),
        notes(notes)
    {}

    friend ostream& operator<<(ostream& stream, const Grid2dNode* node)
    {
        stream << "Grid2dNode "<< hex << reinterpret_cast<long>(node) << dec << " {" << '\n';
        stream << "  l: " << node->location << " c: " << node->cost <<
            " n: " << node->notes << "\n}";
        return stream;
    }

    unsigned int cost;
    const Grid2dLocation location;
    void* notes;
};

} // namespace srs

#endif // GRID2DNODE_HPP_

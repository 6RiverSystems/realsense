/*
 * (c) Copyright 2017 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/search/AStar.hpp>
#include <srslib_framework/search/graph/mapstack/MapStackNode.hpp>

namespace srs {

class MapStack;

class MapStackAStar :
    public AStar<MapStackNode>
{
public:

    MapStackAStar(MapStack* stack, const MapStackSearchParameters& searchParameters = {});
    ~MapStackAStar();

    bool search(const Position& start, const Position& goal);

protected:

    virtual void getExploredNodes(MapStackNode*, std::vector<MapStackNode*>& ) final;

private:

    MapStack*                   stack_              { nullptr };
    MapStackSearchParameters    searchParameters_   { };

};

} // namespace srs

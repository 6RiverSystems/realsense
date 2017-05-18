/*
 * (c) Copyright 2017 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srslib_framework/datastructure/Position.hpp>
#include <srslib_framework/search/AStar.hpp>

namespace srs {

class MapStack;

class MapStackAStar :
    public AStar
{
public:

    struct SearchParameters
    {
        SearchParameters() :
            allowUnknown(false),
            costMapRatio(1.0)
        {}

        bool allowUnknown;
        float costMapRatio;
    };

    MapStackAStar(MapStack* stack, const SearchParameters& searchParameters = {});
    ~MapStackAStar();

    bool search(const Position& start, const Position& goal);

    bool goalReached( );
    
private:

    MapStack*           stack_              { nullptr };
    SearchParameters    searchParameters_   { };

};

} // namespace srs

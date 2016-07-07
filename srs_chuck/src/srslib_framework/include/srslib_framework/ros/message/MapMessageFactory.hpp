/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef MAPMESSAGEFACTORY_HPP_
#define MAPMESSAGEFACTORY_HPP_

#include <srslib_framework/MsgMap.h>

#include <srslib_framework/localization/Map.hpp>

namespace srs {

struct MapMessageFactory
{
    /**
     * @brief Convert a MsgMapConstPtr type into a Map.
     *
     * @param message MsgMap to convert
     *
     * @return Map generated from the specified MsgMap
     */
    static Map* msg2Map(srslib_framework::MsgMapConstPtr message)
    {
        Map* map = new Map(message->info.width, message->info.height, message->info.resolution);
        map->setGrid(message->costs, message->notes);

        return map;
    }
};

} // namespace srs

#endif // MAPMESSAGEFACTORY_HPP_

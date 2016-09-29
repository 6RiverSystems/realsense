/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPMAP_HPP_
#define ROSTAPMAP_HPP_

#include <srslib_framework/Map.h>

#include <srslib_framework/localization/map/Map.hpp>

#include <srslib_framework/ros/tap/RosTap.hpp>
#include <srslib_framework/ros/message/MapMessageFactory.hpp>

namespace srs {

class RosTapMap :
    public RosTap
{
public:
    RosTapMap():
        RosTap("/internal/state/map/complete", "Global Map Tap"),
        map_(nullptr)
    {}

    ~RosTapMap()
    {
        disconnectTap();
    }

    Map* getMap() const
    {
        return map_;
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe(getTopic(), 1, &RosTapMap::onMap, this);

        return true;
    }

private:
    void onMap(const srslib_framework::MapConstPtr& message)
    {
        if (map_)
        {
            delete map_;
        }

        map_ = MapMessageFactory::msg2Map(message);
    }

    Map* map_;
};

} // namespace srs

#endif // ROSTAPMAP_HPP_

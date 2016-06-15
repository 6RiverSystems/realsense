/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPMAP_HPP_
#define ROSTAPMAP_HPP_

#include <string>
using namespace std;

#include <nav_msgs/GetMap.h>

#include <srslib_framework/CompleteMap.h>
using namespace srslib_framework;

#include <srslib_framework/localization/Map.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapMap :
    public RosTap
{
public:
    RosTapMap():
        RosTap("Global Map Tap"),
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
        rosSubscriber_ = rosNodeHandle_.subscribe("/srsnode_map_server/map_complete",
            1, &RosTapMap::onMap, this);

        return true;
    }

private:
    void onMap(const CompleteMapConstPtr& message);

    Map* map_;
};

} // namespace srs

#endif // ROSTAPMAP_HPP_

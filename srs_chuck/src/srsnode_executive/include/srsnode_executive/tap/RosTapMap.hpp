/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#ifndef ROSTAPJOY_HPP_
#define ROSTAPJOY_HPP_

#include <string>
using namespace std;

#include <ros/ros.h>
#include <nav_msgs/GetMap.h>

#include <srslib_framework/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/ros/RosTap.hpp>

namespace srs {

class RosTapMap :
    public RosTap
{
public:
    RosTapMap(string nodeName);

    ~RosTapMap()
    {
        disconnectTap();
        delete currentMap_;
    }

    Grid2d* getCurrentMap()
    {
        setNewData(false);
        return currentMap_;
    }

protected:
    bool connect()
    {
        rosSubscriber_ = rosNodeHandle_.subscribe("/map", 10, &RosTapMap::onMap, this);
        return true;
    }

private:
    //Grid2d* currentMap_;

    Grid2d* convertArrayToGrid(int width, int height, unsigned char& array[]);

    void onMap(const nav_msgs::OccupancyGridConstPtr& message);
};

} // namespace srs

#endif // ROSTAPJOY_HPP_

#include <ros/ros.h>

#include <srsnode_map_server/MapServer.hpp>

int main(int argc, char** argv)
{
    // Create the map server unit and run it
    srs::MapServer mapServer("srsnode_map_server", argc, argv);
    mapServer.run();

    return 0;
}

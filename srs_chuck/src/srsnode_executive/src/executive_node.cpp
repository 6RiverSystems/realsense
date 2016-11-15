/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_executive/Executive.hpp>

int main(int argc, char** argv)
{
    // Create the map server unit and run it
    srs::Executive executive("srsnode_executive", argc, argv);
    executive.run();

    return 0;
}

/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */

#include <srsnode_midbrain/Midbrain.hpp>

int main(int argc, char** argv)
{
    // Create the midbrian and run it
    srs::Midbrain midbrain("srsnode_midbrain", argc, argv);
    midbrain.run();

    return 0;
}

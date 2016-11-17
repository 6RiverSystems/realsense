/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srsnode_navigation/global_planner/PotentialCalculator.hpp>

namespace srs {

class QuadraticCalculator : public PotentialCalculator
{
public:
    QuadraticCalculator(int nx, int ny): PotentialCalculator(nx,ny)
    {}

    float calculatePotential(float* potentials, unsigned int cost, int n, float prev_potential);
};

}


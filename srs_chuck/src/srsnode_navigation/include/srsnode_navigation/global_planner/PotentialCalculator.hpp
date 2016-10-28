/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <algorithm>

namespace srs {

class PotentialCalculator
{
public:
    PotentialCalculator(int nx, int ny)
    {
        setSize(nx, ny);
    }

    virtual ~PotentialCalculator()
    {}

    virtual float calculatePotential(float* potential,
        unsigned char cost,
        int n,
        float prev_potential = -1);

    virtual void setSize(int nx, int ny)
    {
        nx_ = nx;
        ny_ = ny;
        ns_ = nx * ny;
    }

protected:
    inline int toIndex(int x, int y)
    {
        return x + nx_ * y;
    }

    int nx_;
    int ny_;
    int ns_;
};

}

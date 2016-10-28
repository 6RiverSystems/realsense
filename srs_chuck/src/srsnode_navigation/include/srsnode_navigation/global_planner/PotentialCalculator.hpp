/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

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
        float prev_potential = -1)
    {
        if (prev_potential < 0)
        {
            float min_h = std::min( potential[n - 1], potential[n + 1] );
            float min_v = std::min( potential[n - nx_], potential[n + nx_]);

            prev_potential = std::min(min_h, min_v);
        }

        return prev_potential + cost;
    }

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

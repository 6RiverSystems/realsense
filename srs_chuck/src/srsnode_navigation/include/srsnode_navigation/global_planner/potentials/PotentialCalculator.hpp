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
    static const float MAX_POTENTIAL;

    PotentialCalculator(int nx, int ny)
    {
        setSize(nx, ny);
    }

    virtual ~PotentialCalculator()
    {}

    virtual float calculatePotential(float* potentials,
        unsigned int cost,
        int n,
        float prev_potential = -1);

    virtual void setSize(int nx, int ny)
    {
        nx_ = nx;
        ny_ = ny;
        ns_ = nx * ny;
    }

protected:
    int nx_;
    int ny_;
    int ns_;
};

}

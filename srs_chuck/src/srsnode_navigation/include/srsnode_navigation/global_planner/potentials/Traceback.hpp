/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
using namespace std;

#include <srsnode_navigation/global_planner/potentials/PotentialCalculator.hpp>

namespace srs {

class Traceback {
public:
    Traceback(PotentialCalculator* potentialCalculator) :
        pCalculator_(potentialCalculator)
    {}

    virtual ~Traceback()
    {}

    inline int getIndex(int x, int y)
    {
        return x + y * xs_;
    }

    virtual bool getPath(float* potentials,
        double start_x, double start_y, double end_x, double end_y,
        vector<pair<float, float> >& path) = 0;

    void setLethalCost(unsigned char lethal_cost)
    {
        lethal_cost_ = lethal_cost;
    }

    virtual void setSize(int xs, int ys)
    {
        xs_ = xs;
        ys_ = ys;
    }

protected:
    int xs_;
    int ys_;

    unsigned char lethal_cost_;
    PotentialCalculator* pCalculator_;
};

} // namespace srs

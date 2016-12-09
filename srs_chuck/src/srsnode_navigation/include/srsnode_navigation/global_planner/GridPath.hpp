/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <srsnode_navigation/global_planner/Traceback.hpp>
#include <srsnode_navigation/global_planner/PotentialCalculator.hpp>

namespace srs {

class GridPath : public Traceback
{
public:
    GridPath(PotentialCalculator* p_calc):
        Traceback(p_calc)
    {}

    bool getPath(float* potential,
        double start_x, double start_y, double end_x, double end_y,
        std::vector<std::pair<float, float> >& path);

private:
    inline bool checkP(int x0, int y0, float* potentials, float& minValue)
    {
        int index = getIndex(x0, y0);
        if (potentials[index] < minValue)
        {
            minValue = potentials[index];
            return true;
        }

        return false;
    }
};

}

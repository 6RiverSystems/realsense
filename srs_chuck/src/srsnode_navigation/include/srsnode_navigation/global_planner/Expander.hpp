/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <costmap_2d/costmap_2d.h>

#include <srslib_framework/datastructure/graph/grid2d/Grid2d.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

#include <srsnode_navigation/global_planner/PotentialCalculator.hpp>

namespace srs {

class Expander
{
public:
    Expander(LogicalMap* logicalMap,
        costmap_2d::Costmap2D* costMap,
        PotentialCalculator* pCalculator) :
            unknown_(false),
            lethal_cost_(253),
            neutral_cost_(50),
            factor_(3.0),
            logicalGrid_(logicalMap->getGrid()),
            costGrid_(costMap->getCharMap()),
            pCalculator_(pCalculator),
            cells_visited_(0)
    {
        nx_ = costMap->getSizeInCellsX();
        ny_ = costMap->getSizeInCellsY();
        ns_ = nx_ * ny_;
    }

    virtual ~Expander()
    {}

    virtual bool calculatePotentials(
        double start_x, double start_y,
        double end_x, double end_y,
        int cycles, float* potential) = 0;

    void setLethalCost(unsigned char lethal_cost)
    {
        lethal_cost_ = lethal_cost;
    }

    void setNeutralCost(unsigned char neutral_cost)
    {
        neutral_cost_ = neutral_cost;
    }

    void setFactor(float factor)
    {
        factor_ = factor;
    }

    void setHasUnknown(bool unknown)
    {
        unknown_ = unknown;
    }

    void clearEndpoint(float* potentials, int gx, int gy, int s)
    {
        int startCell = toIndex(gx, gy);

        for (int i =- s; i <= s; i++)
        {
            for(int j=-s;j<=s;j++)
            {
                int n = startCell + i + nx_ * j;
                if (potentials[n] < POT_HIGH)
                {
                    continue;
                }

                float c = costGrid_[n] + neutral_cost_;
                float potential = pCalculator_->calculatePotential(potentials, c, n);

                potentials[n] = potential;
            }
        }
    }

protected:
    inline void index2Coordinates(int index, int& x, int& y)
    {
        x = index % nx_;
        y = index / nx_;
    }

    inline int toIndex(int x, int y)
    {
        return x + nx_ * y;
    }

    int nx_, ny_, ns_;

    bool unknown_;
    unsigned char lethal_cost_;
    unsigned char neutral_cost_;
    int cells_visited_;
    float factor_;

    PotentialCalculator* pCalculator_;
    Grid2d* logicalGrid_;
    unsigned char* costGrid_;
};

}

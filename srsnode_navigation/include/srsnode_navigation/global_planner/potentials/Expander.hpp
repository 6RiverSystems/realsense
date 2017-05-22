/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <costmap_2d/costmap_2d.h>

#include <srslib_framework/datastructure/graph/grid2d/WeightedGrid2d.hpp>
#include <srslib_framework/localization/map/logical/LogicalMap.hpp>

#include <srsnode_navigation/global_planner/potentials/PotentialCalculator.hpp>

namespace srs {

class Expander
{
public:
    Expander(LogicalMap* logicalMap,
        costmap_2d::Costmap2D* costMap,
        PotentialCalculator* pCalculator) :
            costGrid_(costMap->getCharMap()),
            logicalGrid_(logicalMap->getGrid()),
            nx_(costMap->getSizeInCellsX()),
            ny_(costMap->getSizeInCellsY()),
            ns_(nx_*ny_),
            pCalculator_(pCalculator)
    {
    }

    virtual ~Expander()
    {}

    void allowUnknown(bool value)
    {
        allowUnknown_ = value;
    }

    virtual bool calculatePotentials(
        double start_x, double start_y,
        double end_x, double end_y,
        int cycles, float* potential) = 0;

    void setLethalCost(unsigned int lethal_cost)
    {
        lethalCost_ = lethal_cost;
    }

    void setNeutralCost(unsigned int neutral_cost)
    {
        neutralCost_ = neutral_cost;
    }

    void setWeightRatio(unsigned int ratio)
    {
        weightRatio_ = ratio;
    }

    void setLogicalCostRatio(unsigned int ratio)
    {
        logicalCostRatio_ = ratio;
    }

    void clearEndpoint(float* potentials, int gx, int gy, int s)
    {
        int startCell = coordinates2Index(gx, gy);

        for (int i =- s; i <= s; i++)
        {
            for (int j = -s; j <= s; j++)
            {
                int n = startCell + i + nx_ * j;
                if (potentials[n] < PotentialCalculator::MAX_POTENTIAL)
                {
                    continue;
                }

                float c = costGrid_[n] + neutralCost_;
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

    inline int coordinates2Index(int x, int y)
    {
        return x + nx_ * y;
    }

    bool allowUnknown_			{ true };

    unsigned char* costGrid_		{ nullptr };

    unsigned int lethalCost_		{ 253 };
    WeightedGrid2d* logicalGrid_	{ nullptr };
    unsigned int logicalCostRatio_	{ 100 };

    int nx_				{ -1 };
    int ny_				{ -1 };
    int ns_				{ -1 };
    unsigned int neutralCost_		{ 50 };

    PotentialCalculator* pCalculator_	{ nullptr };

    unsigned int weightRatio_		{ 100 };
};

}

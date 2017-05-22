/*
 * (c) Copyright 2015-2016 River Systems, all rights reserved.
 *
 * This is proprietary software, unauthorized distribution is not permitted.
 */
#pragma once

#include <vector>
#include <algorithm>

#include <srslib_framework/datastructure/graph/grid2d/WeightedGrid2d.hpp>

#include <srsnode_navigation/global_planner/potentials/Expander.hpp>

namespace srs {

struct Index
{
    Index(int a, float b) :
        i(a),
        cost(b)
    {}

    int i;
    float cost;
};

struct GreaterThan
{
    bool operator()(const Index& a, const Index& b) const
    {
        return a.cost > b.cost;
    }
};

class AStarExpansion :
    public Expander
{
public:
    AStarExpansion(LogicalMap* logicalMap, costmap_2d::Costmap2D* costMap,
        PotentialCalculator* pCalculator);

    virtual bool calculatePotentials(
        double start_x, double start_y,
        double end_x, double end_y,
        int cycles,
        float* potentials) override;

private:
    void add(float* potentials,
        float prev_potential,
        int next_i, WeightedGrid2d::BaseType weight,
        int end_x, int end_y);

    std::vector<Index> queue_;
};

}
